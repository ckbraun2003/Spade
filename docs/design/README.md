# `design-specs/spade/` -- the Spade component directory

Spade is the simulation engine beneath Kat: a fixed-step, deterministic physics core with a
CPU reference twin for every authoritative pass, a Vulkan compute backend, and a CPU rasterizer.
**`SL1` states normatively that Spade is a domain-neutral blanket engine of which Kat is one
consumer** -- not "Kat's simulation engine", which is how older documents framed it.

This directory is the consolidation of five specifications and two as-built records into eight
subject-organized documents. It was created 2026-09-07 under the estate's consolidation program
(`kat_progress.md` section 16.7), which ruled: *do the part that removes a second home for a
fact, and sequence the part that removes content.*

> **The ruling-id vocabulary is unchanged.** Source code, other realms' specs, and test comments
> cite `SL7`, `HS2 rule 2`, `engine D7`, `RS1a`, `SA2`. Every identifier survives verbatim; only
> the *organization* changed -- from chronological (one document per session) to by-subject.
> `RECONCILIATION.md` maps every identifier to its new home.

---

## Read in this order

| # | Document | What it answers |
|---|---|---|
| 1 | **`01-charter.md`** | What Spade *is*, where its boundary runs, and the commitments everything else inherits |
| 2 | **`02-engine.md`** | The engine core: the step model, the ten-pass substep schedule, state, many-worlds, snapshot, physics, compute backends, parity |
| 3 | **`03-world-and-render.md`** | The world file format (schema v2) and how a world becomes pixels |
| 4 | **`04-objects.md`** | The object graph, components, and declared behaviors |
| 5 | **`../integration/spade-c5-adapter.md`** | The C5 seam -- the contract between Kat and Spade |
| 6 | **`06-sandbox-and-v1.md`** | The sandbox, the v1 transfer audit, and v1's retirement |
| 7 | **`07-status.md`** | **Specified vs built.** The only document describing the present |
| 8 | **`08-lessons.md`** | Engine-specific engineering knowledge earned during execution |
| -- | **`RECONCILIATION.md`** | Where every ruling went, and how each contradiction was resolved |
| -- | **`superseded/`** | The seven original documents, verbatim |

**If you are here to do one thing:**

- *Change the engine* -> `01` for the constraints you inherit, then `02`.
- *Add a component or behavior* -> `04`, then `spade/engine/objects/README.md` (tracked).
- *Touch the C5 boundary* -> `05` for what Spade implements, `../runtime/04-host-c5.md` for what
  C5 means. Read `05`'s opening block first -- it maps the three layers.
- *Retire or quarantine v1* -> `06`. **It is blocked; the block is real.**
- *Find out what actually exists today* -> `07` only. Nothing else is maintained forward.

---

## Which document owns which ruling series

| Series | Owner | Notes |
|---|---|---|
| `P1`-`P8` (pillars) | `01-charter.md` | Foundation charter, approved 2026-08-06 |
| `SA1`-`SA3` | `01-charter.md` | The three errata amendments |
| `SL1`, `SL2`, `SL2a`, `SL2b` | `01-charter.md` | Identity, library surface, release identity |
| `D1`-`D12` | `02-engine.md` | Engine decision record, approved 2026-08-08 |
| `A1`-`A11` | `02-engine.md` | Addendum A, signed 2026-08-10. Folded in by subject, not appended |
| `SL6` | **split** | The *schedule* half (the two slots and their positions) is in `02-engine.md`; the *behavior registry* half is in `04-objects.md`. One rule, two subjects -- neither restates the other |
| `RS1`-`RS15` | `03-world-and-render.md` | Except `RS10a/b/c`, which are in `06` |
| `SL3`, `SL4`, `SL5` | `04-objects.md` | The object model |
| `HS1`-`HS12`, `F-1`-`F-6` | `../integration/spade-c5-adapter.md` | Plus errata `R1`, quoted for C5's origin |
| `SL7`-`SL15` | `06-sandbox-and-v1.md` | Transfer audit, sandbox, quarantine, CI |
| `SL16`, `SL17`, `SL18` | `06-sandbox-and-v1.md` | Vocabulary (s.8), the P0-P7 phase table (s.6), the nine verification obligations (s.7). ⭐ **Harvested 2026-09-17** -- until then these three lived **only** in the superseded 24th spec, so signing `SL1`-`SL18` would have ratified three rulings with no live text |
| `S1`-`S8` (phases) | `07-status.md` | Status only; the phase *definitions* live in `02` and `03` |

**Deliberately stated once.** These were each stated three or four times across the originals;
duplication is what produced most of this estate's contradictions.

- The **three determinism grades** -> `01-charter.md`. (Was in four places.)
- The **kat-free boundary** -> `01-charter.md`, naming all three enforcement points.
  `../integration/spade-c5-adapter.md` carries only the C5-facing half and points back.
- **v1's disposition path list** -> `06-sandbox-and-v1.md`. (Was in two, with opposite verbs.)

---

## Status -- read this before citing anything

| Spec | Series | Status |
|---|---|---|
| Foundation charter (7th) | `P`, `SA` | **Approved** v1.0, 2026-08-06 |
| Engine design (9th) | `D`, `S`, `A` | **Approved** v1.0, 2026-08-08; Addendum A signed 2026-08-10 |
| Sim host (17th) | `HS`, `F` | **Signed off** 2026-08-19 |
| Rendering and scene (21st) | `RS` | ✅ **SIGNED 2026-09-17** -- the enumerated **20**; ⛔ `RS10b` EXCLUDED |
| Library and sandbox (24th) | `SL` | ✅ **SIGNED 2026-09-17** -- ~30 ids |

> ✅ **The chain's weak link — CLOSED 2026-09-17.** The two documents carrying the most amendment
> weight over the approved engine design -- the 21st and 24th -- were **both drafts** for three
> weeks. The S7 split, the world-schema v1 -> v2 lift, the `D10` endpoint change and the
> eight-to-ten schedule growth rested on unapproved text while **already built and shipping**. It
> was a documentation debt, not an implementation one, and the user discharged it.
>
> ⚠ **One amendment remains genuinely owed and the signature did not close it:** the approved
> engine design **still reads S7 as one undivided Vulkan phase**. `RS1` governs because it
> describes what shipped, and `RS14` books that amendment -- see s.13's table. **Signing the `RS`
> series ratified the amendment; it did not go back and edit the approved page.** Those are
> different acts and only one of them has happened.

---

## How to cite

Cite qualified, because the identifier namespaces overlap across the estate:

- **`engine D7`**, not bare `D7` -- other specs use `D` too.
- **`seat S4`** vs **`engine S4`** -- the spec-seat register and the engine build phases both use
  `S1`-`S8`. Always qualify.
- **`charter P3`** vs an SL phase -- see `RECONCILIATION.md` `C9`; the SL phase labels were
  renamed here precisely because they collided with the charter's pillars.
- **`SPIR-V rule P1`** is a third `P` namespace (scan rules), local to `02-engine.md` section on
  compute.

---

## Foreign identifiers -- where they are defined

These documents cite ~63 identifiers owned by other realms. This table says **where each series
lives**, not what each ruling says -- the owning realm defines its own, and those realms are being
consolidated by their own sessions. Rows point at *directories* where a realm has one, so they
survive that realm's reorganization.

| Prefix | Series | Defined in |
|---|---|---|
| `B1`-`B26` | amendment batches | `../archive/spec-corpus-v1/kat-spec-amendment-batch-2.html` (`B1`-`B14`) / `-3` (`B15`-`B19`) / `-4` (`B20`-`B26`) |
| `B27` | `kat_set_host_caps` -- **not from any batch**; the runtime realm homes it | `../runtime/02-abi.md` s.3 (contracts spec s.3) |
| `C1`-`C5` | **the four platform contracts + C5** -- `C1` .kpkg, `C2` C ABI, `C3` component, `C4` trainer, `C5` sim host | `../runtime/superseded/kat-runtime-sdk-contracts.html` (**moved 2026-09-08** by the runtime realm's consolidation; this table pointed at the old flat path until then), live at `../runtime/02-abi.md`. **`C5` finally has a section, 2026-09-08**: `../runtime/04-host-c5.md` |
| `CG1`-`CG13` | commercial gates | `../archive/spec-corpus-v1/kat-commercial-analysis.html` |
| `CN-1`-`CN-7` | consolidation rulings | `../kat_progress.md` section 16.7 |
| `CP1`-`CP22` | component catalog | `../components/` |
| `CS1`-`CS8` | default content | `../archive/spec-corpus-v1/kat-default-content.html` |
| `DM1`-`DM12` | demo mission packages | `../components/` |
| `G1`-`G15` | configuration architecture | `../archive/spec-corpus-v1/kat-configuration-architecture.html` |
| `MN-`, `IM-`, `CR-` | 2026-08-07 spec review findings | `../archive/reviews/kat-spec-review-2026-08-07.md` |
| `MV1`-`MV7` | viewport and cameras | `../editor/` |
| `R1`-`R15` | errata | `../archive/spec-corpus-v1/kat-spec-errata.html` |
| `RI-1`, `RI-2` | open user rulings, **dissolved by `TS17`** 2026-08-26 | `../archive/registers/kat-open-agendas.md` |
| `SC1`-`SC11` | repo scaffolding | `../archive/spec-corpus-v1/kat-repo-scaffolding.html` |
| `SD1`-`SD8` | customer SDK | `../sdk/` |
| `T0`-`T3` | test tiers | `../kat-testing-spec.html` |
| `TA1`-`TA6` | editor technical spec | `../editor/` |
| `TR1`-`TR18`, `TR-A1`-`A4` | training system | `../components/` |
| `TS1`-`TS22` | training structures | `../components/` |
| `VC-`, `VI-`, `VM-` | 2026-08-09 verification review findings | `../archive/reviews/kat-spec-review-2026-08-09.md` |

### Three collisions to know about

- **`C5`** means the sim-host *contract* everywhere except `RECONCILIATION.md`, where `C1`-`C11` are
  that file's own contradiction ledger. Contract `C5` is always written as "C5" in prose; the ledger's
  are always written `` `C5` `` inside `RECONCILIATION.md` and nowhere else.
- **`P`** has three meanings: charter pillars `P1`-`P8` (`01-charter.md`), SPIR-V scan rules `P1`-`P5`
  (`02-engine.md`, always written "SPIR-V rule `P1`"), and the 24th spec's phases (always written
  with their owner, "`SL17`'s Phase P5", never bare).
- **`S1`-`S8`** are engine build phases here. The spec-seat register uses the same numbers -- cite
  those as "seat S4".

### Locally defined, easy to mistake for foreign

`E1`/`E2` (SPIR-V scan rules, `02-engine.md`) and `D-S5-1`/`D-S6-2` (in-program execution decisions,
`07-status.md`) are this realm's own.

## Every marker in this directory

**27 annotation blocks** across `01`-`07` (`08-lessons.md` carries none). Grouped by what a reader
must **do**.

> **The count was wrong until 2026-09-08.** This section said *"22 annotation blocks"* and then
> listed groups summing to 21 -- two numbers, neither matching the other and neither matching the
> documents. The list had dropped `06`'s *Path disagreement* block, `05`'s `STATUS SUPERSEDED`,
> one of `01`/`02`'s `STALE` blocks, and it counted `C4`/`C10` as one block each where each is two.
> **A hand-maintained index of a growing set decays silently**, because nothing recomputes it and a
> plausible number reads exactly like a correct one. The enumeration below was regenerated from the
> files; if you add a marker, add its row here in the same change.
>
> **It drifted again within the hour.** Correcting a withdrawn finding added a `WITHDRAWN` block to
> `07`, and that edit did not think of itself as an index change either -- which is the whole point.
> Caught only because the count was re-derived rather than re-read. **Re-derive it; do not trust
> this paragraph.**

### A note on checking the references in these documents

These ten files carry **120 distinct path-shaped references**. Half are written as bare filenames
(`graph.hpp`, `scenes.cpp`) -- prose shorthand, not links, and not checkable without a search. Of the
60 written as paths, most are relative to `spade/engine/`, which is this realm's reading convention.

Two things a reference check here must not do:

1. **Report coverage as correctness.** The check I ran all session matched 13 of the 120 -- **11%** --
   and printed *"0 missing"* the whole time. Widening it to the full population surfaced 19 candidates,
   of which exactly **two** were real: `../kat-runtime-sdk-contracts.html` (the runtime realm's
   consolidation moved it under `runtime/superseded/` while my link still pointed at the flat path) and
   a `spade/engine/tests/...` path that never existed. **"0 broken" over 11% and over 100% print
   identically.**
2. **Treat a missing file as a broken reference.** This directory records deletions, retirements and
   quarantines, so **naming a file that does not exist is frequently the correct behaviour**: `RS1a`
   names `dronesim/spade/raster.cpp` *because it was deleted* (verified: added in `837b97e6`, since
   removed; `dronesim/fake/raster.cpp` is a different, live file), `SL9f` names `Barycentric.geom`
   as retired-with-reason, and `SL14a` describes a quarantine that must not execute. A checker cannot
   tell a broken link from a deliberately recorded absence. **I nearly "fixed" `RS1a` into a false
   statement.** Resolve each candidate against `git log --diff-filter=A` before believing it.

⚠ **Expect more of the first kind.** The `Defined in` column above points at flat `../kat-*.html`
files, and every realm consolidating this week moves its originals into `<realm>/superseded/`. Those
links rot as peers land, not as this realm changes.

### Needs a decision or carries a live constraint -- 2

| Marker | Where | What |
|---|---|---|
| `BLOCKED` | `06-sandbox-and-v1.md:77` | **The v1 quarantine must not execute.** SPH is the one open transfer row; v1 is still the only fluid implementation. The estate's single most important operational fact |
| `SPEC vs BUILT` | `04-objects.md:127` | `SL4` says attach/detach are queued to step boundaries; the built graph mutates immediately. Harmless today because `Simulation` holds no `ObjectGraph` -- **becomes live the moment `SubstepContext` gains one** |

### Settled, kept as the record of how -- 17

| Marker | Where | What it settles |
|---|---|---|
| `GAP CLOSED` | `05` | C5's missing contracts-spec section, written by the runtime realm as `../runtime/04-host-c5.md` on 2026-09-08. Kept because it now carries the three-layer split and two open findings, not because anything is owed |
| `CLOSED` | `07` | The same closure, struck from the blockers table |
| `WITHDRAWN` | `07` | *"C5's semver has never moved"* -- every fact true, the conclusion false. Held the blockers row for part of 2026-09-08 before `git log -S` killed it |
| `SUPERSEDED BY WHAT SHIPPED (C4)` | `02` | `D10`'s strangler window -- approved text overtaken by S7a |
| `OPERATIVE (C4)` | `03` | The other half: `RS1` governs, because it describes what shipped |
| `SUPERSEDED BY WHAT SHIPPED (C10)` | `02` | The approved render-module paragraph |
| `OPERATIVE, with a bookkeeping gap (C10)` | `03` | Its live counterpart, and the gap that remains |
| `SUPERSEDED by A1` | `02` | Camera sensors moved off the render queue |
| `STATUS SUPERSEDED (TS11)` | `05` | `F-5`'s *experimental / non-normative* status, withdrawn by the 22nd spec |
| `SUPERSEDED` | `06` | `RS10b`'s "deleted" replaced by `SL14a`'s quarantine |
| `Path disagreement, resolved against the tree` | `06` | `RS10b` names a path the tree does not have |
| `RESOLVED (C6)` | `01` | External repo is a live upstream, per the later user ruling |
| `FIXED (C11)` | `06` | The `SL9f` register row, **fixed and mutation-tested 2026-09-07** |
| `NOT THE CURRENT STATE` | `03` | The executable register describes the end of the RS program, not today |
| `RECONCILED` | `04` | Ten-vs-nine component types |
| `RECONCILED` | `06` | `RS10c` and `SL11`/`SL12` are different ImGui surfaces |
| `SPEC vs BUILT` | `04` | Serialization by name rather than id -- the built form is the stronger one |

### Archaeology -- 8

Eight `STALE` blocks: two in `01`, three in `02`, one in `03`, two in `05`. Each marks a passage in a
**superseded** original as out of date. Nobody is instructed to read those; the markers exist so that a reader who opens the
archive for provenance is not misled by it. No action.

---

## Open and owed

Full detail in `07-status.md` section 6. The four that block something:

1. **SPH fluid is the one open transfer-register row.** v1 remains the only implementation of the
   fluid solver, so **the v1 quarantine must not execute.** Plan B is unwritten.
2. ✅ **`RS` and `SL` are SIGNED** (2026-09-17) -- closed. The enumerated lists, not the ranges;
   ⛔ `RS10b` excluded as superseded rather than ratified. Record: `07-status.md` s.3.1.
4. **No guard ties a header's ABI version constant to its surface.** C5's minor has correctly
   never moved and C2's 0->1 for `B27` was correctly done -- but both by attention, not by
   construction: no check would fail if the next additive call skipped its bump. Reaches outside
   Spade; SDK holds the C2 half, `../runtime/07-status.md` s.1 the C5 half.
   *(Item 4 has now been replaced twice in one day. It first read "C5 has no section in the
   contracts spec" -- **closed** when the runtime realm wrote `../runtime/04-host-c5.md`. It then
   briefly read "C5's semver has never moved", which was **withdrawn**: true facts, false
   inference -- see the `WITHDRAWN` block in `07-status.md` s.6.)*

---

## Relationships to other realms

Spade does not stand alone. These are the live connections, with the ruling that creates each:

| Realm | Connection |
|---|---|
| **Runtime / SDK** | **C5** is the seam -- `dronesim` implements it, Spade sits beneath and never implements it (errata `R1`). Since 2026-09-08 C5 is defined in **three layers**: the tracked header `dronesim/include/kat/kat_host_abi.h` (signatures), `../runtime/04-host-c5.md` (semantics, semver, conformance obligations), and `../integration/spade-c5-adapter.md` (what Spade implements). No layer copies another. `SD1`/`SD6` make Spade CPU-vs-Vulkan device parity an SDK MVP requirement |
| **Configuration** | **`G2`** ratifies that **the world format is Spade's**, and removed `world.schema.json` from the editor side. The `kathost_create` opaque backend block is schema-declared and validated by the backend, and **nothing in it may change stepping results** |
| **Training** | **`TS11`** promotes `kathost_x_batch_*` to the ratified C5 training seam (amending `F-5` from outside). **`B16`** struck the Python-native interim adapter: rollouts run on the STEPPED runtime + Spade CPU via C5, and a bench miss triggers design review, never a second physics. The route generator mirrors Spade's `splitmix64` domain-separation constants exactly |
| **Editor** | Uses Spade through C5 only. **`TA1`**: `kathost_step` never renders, and stepping must be bit-identical for any render cadence including none. The editor viewport is Spade's raster. `spade/engine` is under a standing do-not-edit rule for the editor's design work |
| **Component catalog** | **`HS8a`** is a jointly-owned CP-series amendment (the airframe descriptor extension). **`CP8`/`CP18`** file the Y-up (Spade) vs Z-up (Kat) axis reconciliation |
| **Testing** | Spade runs its own CI legs from first commit (an explicit `D11` carve-out) and selects on a bare `spade` ctest label outside the `T0`-`T3` vocabulary. GPU parity and bench run on the dev box, outside the tier model entirely. ⚠ *Carrier updated 2026-09-18: those legs were `.github/workflows/spade.yml` until hosted CI was deleted; they are now the `spade-linux` and `spade-windows` sections of `docker/run-ci-linux.sh` and `docker/run-ci-windows.ps1`, each configuring, building, `ctest -L spade` and running the consumer smoke against `spade/build-ci`. **`D11` itself is unchanged** -- only the file it lives in moved* |
| **Docs / repo** | `spade/` is an enumerated infrastructure-directory exemption; adding a member is a spec change. **No tracked file may carry a relative markdown link into `design-specs/`** -- refer to it as plain text |

---

## Provenance

Harvested 2026-09-07 from seven documents totalling ~258 KB, now in `superseded/`:

`superseded/kat-spade-simstack-foundation.html` - `superseded/kat-spade-engine-design.html` - `superseded/kat-spade-host.html` -
`superseded/kat-spade-render-scene.html` - `superseded/kat-spade-library-sandbox.html` -
`superseded/kat-spade-execution-record.md` - `superseded/kat-spade-s1-s4-implementation-record.html`

### The most exposed documents here are the new ones

Stated first because it is the sharpest fact in this section and it took a peer's aside to notice:

> **These ten documents -- 4,428 lines, the live specs a reader is now directed to -- exist in
> exactly one place.** `design-specs/` is gitignored, so they are not in git history; none has been
> published, so there is no artifact. No second copy of any kind exists.

The archive below is in better shape than the live set: seven documents with four artifacts between
them, stale but extant. The consolidation directed every reader away from those and toward ten files
with nothing behind them at all.

This section was written to audit the *superseded* originals and did not, until 2026-09-08, ask the
same question of the documents doing the superseding. That is the inversion this directory
catalogues, committed here: **the audit covered the old artifact and not the new one.**

### Recoverability -- do not delete `superseded/`

Measured 2026-09-07, then **re-measured across the whole corpus** rather than the register alone --
a document can have an artifact whose URL was never written into its own register row, and
"no URL in the register" and "no URL anywhere" are different findings of which only the second is
fatal. All 30 artifact URLs in `design-specs/` were searched. The result held:

| Archived document | Live artifact URL? |
|---|---|
| `kat-spade-simstack-foundation.html` | yes |
| `kat-spade-engine-design.html` | yes |
| `kat-spade-host.html` | yes |
| `kat-spade-s1-s4-implementation-record.html` | yes |
| **`kat-spade-render-scene.html`** | **NO** |
| **`kat-spade-library-sandbox.html`** | **NO** |
| **`kat-spade-execution-record.md`** | **NO** |

**Three of the seven exist nowhere else.** `design-specs/` is gitignored, so those three have no
artifact, no git history and no second copy -- one directory, on one machine. Two of the three are
the **`RS` and `SL` drafts whose sign-off is still owed**, which makes them the least replaceable
documents in the realm and the ones a reader is most likely to need.

### "Backed" does not mean "current" -- verified by reading the artifacts

The four with URLs were checked against the live artifact store, and three of them **lag their local
file**:

| document | artifact updated | local edited | gap |
|---|---|---|---|
| `kat-spade-s1-s4-implementation-record.html` | 2026-08-11 | 2026-08-11 | current |
| `kat-spade-engine-design.html` | 2026-08-09 | 2026-08-10 | **crosses a signing date -- see below** |
| `kat-spade-simstack-foundation.html` | 2026-08-06 | 2026-08-10 | 4 days |
| `kat-spade-host.html` | 2026-08-19 | 2026-09-02 | 14 days |

**The engine-design artifact was read, not inferred. It has no Addendum A.** It ends at section 14
and its footer reads *"v1.0 approved 2026-08-08 ... next: writing-plans for S1-S4"*. The local file
carries **`A1`-`A11`**, signed 2026-08-10 -- eleven rulings including `A1`'s supersession of the
camera-on-the-render-queue reading, `A2`'s frame-pool deferral and `A11`'s version bump.

So the real count is not three:

**The host artifact was read too, and it is worse.** Published 2026-08-19, and it still carries:

- the chip **"DRAFT -- awaiting sign-off"**, though the spec was **signed off 2026-08-19**;
- **`F-5` as "experimental" and "explicitly non-normative"**, with **no mention of `TS11`** -- the
  ruling that promoted `kathost_x_batch_*` to the ratified C5 training seam (user-ruled 2026-08-26,
  stamped locally 2026-09-01);
- section 12's "`F-5` is evidence, not contract", unamended;
- `RI-1`/`RI-2` described as open user rulings, though `TS17` dissolved them 2026-08-26.

**This one misleads other realms, not just this one.** SDK and Training both depend on C5, and the
only published copy tells them the batch surface is non-normative when it has been ratified.

**The charter artifact was read too. Three material staleness defects, all confirmed:**

1. **Its section 4 amendment table uses bare `A1`, `A2`, `A3`** -- the namespace *retired* on
   2026-08-08 (batch 2 section 0) precisely because it collides with engine Addendum A's `A1`-`A11`.
   Both documents are published. **So the published corpus contains the live collision this
   consolidation resolved as `C7`**, and an off-box reader has no way to tell which `A3` is meant.
2. Section 5 still reads "Spade's version stays `0.1.0` until the engine session establishes v2
   versioning" -- `A11` bumped it to 0.2.0 on 2026-08-10.
3. Section 2 still reads Spade "is **Kat's engine, cleanly bounded** ... spends no energy on
   standalone packaging or independent releases" -- both halves falsified by `SL1` and `SL2`, and
   section 5's "the old standalone repo is now the archive" is the `C6` text `SL2a` superseded.

> **Six of seven documents have no verified-current copy off this machine.** Three have no artifact
> at all; the engine design's is missing every Addendum A ruling; the host's contradicts a
> cross-realm ruling and its own sign-off; the charter's carries a retired identifier namespace,
> a superseded version number and a superseded identity statement.
> **Only `kat-spade-s1-s4-implementation-record.html` is current** -- zero dated edits after publish.
>
> **Five of the six were measured by reading the artifact or sweeping the corpus, not inferred.**
> All three artifact reads were **complete** -- each returned the document through its closing
> `</html>`, so no late edit could sit past where the read stopped.

### What publishing actually was

A second realm ran the date comparison across the whole estate: **33 of 35 documents with a
resolvable artifact are stale** (their measurement, screening-grade -- it over-detects, since any
`2026-` date counts, but it cannot under-detect, because a stamp later than the publish date means
the document changed). They also corrected themselves from 1-of-5 to 0-of-5 in the process: they had
called a document current on a **partial** artifact read, and its late edit sat past where they
stopped. **The cheap instrument was the more complete one** -- a date comparison cannot miss a late
edit the way a partial read can.

That reframes the whole question, and it is not a Spade problem:

> **The published set is not a mirror of the design specs. It is a snapshot taken around approval
> time, and the estate has been editing ever since.** 33 of 35 is not a backlog; it is a category
> error about what publishing was doing.

The practical consequence for this realm: republishing the four is worth doing, but it fixes today's
instance rather than the mechanism. Nothing records a publish date next to a document, so
"is this current?" is not merely unchecked -- it is *unaskable* without going to the artifact store.
`Artifact action: list` carries `updated` for every artifact, so the question is answerable today; a
`published:` column beside each register URL would make it cheap.

**The rule, proposed by the components realm and adopted here:** a realm may delete its
`superseded/` only once every member has a live artifact URL *and that artifact has been verified
current*. A URL in a register proves a publish happened once, not that it matches. This realm does
not qualify on either clause.

The four that do have artifacts correspond to the archived originals; they were not edited, so this
consolidation implies no republish.
