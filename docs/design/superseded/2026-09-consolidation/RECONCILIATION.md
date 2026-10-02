# Reconciliation ledger

**What this document is.** The audit trail for the 2026-09-07 consolidation of the Spade estate.
It answers three questions:

1. **Where did ruling `X` go?** -- section 1, so every existing citation still resolves.
2. **What did the sources disagree about, and how was it settled?** -- section 2.
3. **What was removed, and what was found?** -- sections 3 and 4.

Nothing here is normative. It records decisions made *about* the documents, not about the engine.

> **Why this file cites documents and marker names, never line numbers.** It used to. Every one of
> the thirteen line pointers had rotted within a day, because each edit to a sibling document
> shifts them and nothing reports it -- a reference that invalidates itself silently is the same
> defect class this directory keeps cataloguing. Markers are named and few per document, so
> `SUPERSEDED` in `06-sandbox-and-v1.md` stays findable after any edit.

---

## 1 - Identifier map

Every ruling identifier survives verbatim. Only the organization changed.

| Series | Source document (now in `superseded/`) | New home |
|---|---|---|
| `P1`-`P8` | `superseded/kat-spade-simstack-foundation.html` | `01-charter.md` s.2 |
| `SA1`-`SA3` | `superseded/kat-spade-simstack-foundation.html` s.4 | `01-charter.md` s.3 |
| `D1`-`D12` | `superseded/kat-spade-engine-design.html` s.2 | `02-engine.md`, by subject |
| `S1`-`S8` | `superseded/kat-spade-engine-design.html` s.13 | definitions in `02`/`03`; **status in `07-status.md`** |
| `A1`-`A11` | `superseded/kat-spade-engine-design.html` Addendum A | `02-engine.md`, folded in by subject, not appended |
| `HS1`-`HS12`, `HS3a`, `HS8a`, `HS8b` | `superseded/kat-spade-host.html` | `../integration/spade-c5-adapter.md` |
| `F-1`-`F-6` | `superseded/kat-spade-host.html` s.9 | `../integration/spade-c5-adapter.md` |
| `RS1`-`RS9`, `RS1a`, `RS6a`, `RS11`-`RS15`, `RS11a` | `superseded/kat-spade-render-scene.html` | `03-world-and-render.md` |
| `RS10a`, `RS10b`, `RS10c` | `superseded/kat-spade-render-scene.html` s.10 | **`06-sandbox-and-v1.md`** (not `03`) |
| `RS10` (bare) | `superseded/kat-spade-render-scene.html` s.10 heading | Not a ruling -- a container whose entire substance is `RS10a/b/c`. Searching for it lands in `06-sandbox-and-v1.md` |
| `SL1`, `SL2`, `SL2a`, `SL2b` | `superseded/kat-spade-library-sandbox.html` s.1-2 | `01-charter.md` s.1, s.5, s.6 |
| `SL3`, `SL4`, `SL5` | `superseded/kat-spade-library-sandbox.html` s.3 | `04-objects.md` |
| `SL6` | `superseded/kat-spade-library-sandbox.html` s.4 | **split** -- schedule half in `02-engine.md`, registry half in `04-objects.md` |
| `SL7`-`SL9`, `SL9a`-`SL9f` | `superseded/kat-spade-library-sandbox.html` s.5 | `06-sandbox-and-v1.md` |
| `SL10`-`SL15`, `SL14a/b/c`, `SL15a/b` | `superseded/kat-spade-library-sandbox.html` s.6-8 | `06-sandbox-and-v1.md` |
| `SL16` | `superseded/kat-spade-library-sandbox.html` s.9 | `06-sandbox-and-v1.md` **s.8** ⭐ **harvested 2026-09-17** |
| `SL17` | `superseded/kat-spade-library-sandbox.html` s.10 | `06-sandbox-and-v1.md` **s.6** ⭐ **completed 2026-09-17** -- all eight phase rows, the entry condition, and checkpoints ①-④ |
| `SL18` | `superseded/kat-spade-library-sandbox.html` s.11 | `06-sandbox-and-v1.md` **s.7** -- all nine, owner named per row ⭐ **harvested 2026-09-17**; the object-model subset stays at `04-objects.md` s.9 |
| Standing rules 1-12 | `superseded/kat-spade-execution-record.md` s.3 | `07-status.md` s.4 (plus a 13th from Plan A) |
| Forward obligations | `superseded/kat-spade-s1-s4-implementation-record.html` s.6 | `07-status.md` s.6 |
| Task-to-commit traceability | `superseded/kat-spade-s1-s4-implementation-record.html` s.7 | `07-status.md` s.7 |
| Local `A1`-`A3` (demo-scene rules) | `superseded/kat-spade-s1-s4-implementation-record.html` s.3 | **renamed** -- see `C7`. Now `07-status.md` s.4 rule 7 |

> ⚠⚠ **THE `SL16`-`SL18` ROW USED TO READ: `SL16`-`SL18` | … | *"vocabulary and verification folded into
> the documents they govern"*. That was false, and the WAY it was false is worth more than the
> correction.** `SL16` had been folded into nothing at all; `SL17` had lost three of eight phase
> rows, a P7 deliverable and all four checkpoint numbers; `SL18` had lost four of nine obligations.
>
> ⭐⭐⭐ **EVERY OTHER ROW IN THIS TABLE GIVES AN ADDRESS. THAT ONE GAVE A DESCRIPTION.** *"Folded
> into the documents they govern"* names no destination, so **there is nothing a reader can open
> and nothing a link checker can resolve** -- it asserts a completed action in a column whose every
> other cell is a place. It reads as more thorough than `06-sandbox-and-v1.md` because it sounds
> like a principle rather than a pointer, and that is exactly why it survived.
>
> **The rule: a destination column holds destinations.** If a harvest genuinely scatters a ruling
> across several documents, the cell lists all of them -- and if the cell is getting long, that is
> information about the harvest, not a reason to summarise it. A single unverifiable sentence hid
> three incomplete harvests for ten days, in the one table whose whole job is to prove nothing was
> lost.

**`SL6` is the only split ruling.** The schedule half (which two slots exist and why their
*positions* are a parity requirement) belongs with the step model; the registry half (registration
order, GPU eligibility as refusal) belongs with behaviors. Neither document restates the other --
each carries the half it governs and points across.

---

## 2 - Contradiction ledger

Eleven. Five were known going in; six were found during the harvest. **Ten are resolved.** One
stays open, and it is a code defect rather than a documentation question (`C11`).

The rule applied throughout, and it is the one this directory should keep using: **where the code
shipped, the document that describes what shipped is operative** -- a draft that matches reality
outranks an approved page that does not. Where a later *user ruling* exists, it governs. Only a
genuine choice between live alternatives is left open.

### `C1` - v1 is deleted, or v1 is quarantined -- RESOLVED

- `RS10b` (21st, draft): `spade/src/`, `include/`, `examples/`, `assets/`, the `SPADE_BUILD_V1`
  option, the viewer bridge and the vendored GL chain **"are deleted"**. Repeated in `RS11`'s
  register rows and `RS12` checkpoint 4.
- `SL14a` (24th, draft): those same paths **"move to `spade/legacy/` as pure renames with zero
  edits"** -- quarantine, not deletion, completely cut off.

**Quarantine is operative.** SL is later and its banner explicitly claims the amendment. `RS10b`'s
"deleted" is dead text, and it is dangerous dead text because nothing inside the render spec points
forward -- a reader arriving there first gets the wrong instruction. Marked `SUPERSEDED` at
`06-sandbox-and-v1.md`.

Path disagreement **resolved against the tree, in SL's favour**: there is no `spade/tools/`. The
viewer is at `spade/engine/tools/viewer/` (`bridge.cpp`, `bridge.hpp`, `main.cpp`, `scenes.cpp`).
RS also names only `bridge.cpp` where SL moves the whole directory, and SL is right to split them --
`scenes.cpp` is v2 code to harvest, not v1 code to quarantine.

> **And the quarantine has not run.** See `C11` and `07-status.md` s.6 -- it is blocked by the open
> SPH transfer row, correctly.

### `C2` - the ImGui builder: not built here, or built at Phase P5 -- RESOLVED

`RS10c` says the ImGui world/scene builder is "a named seat, not built here" and that the HUD
"arrives with S7b". `SL11`/`SL12` and SL's Phase P5 build an ImGui shell with hierarchy, inspector
and asset browser.

**Different surfaces.** The sandbox's ImGui *shell* is SL's; Plan C is written (2026-09-17) and unexecuted. The ImGui
*debug HUD on the GPU path* remains S7b's. Marked `RECONCILED` in `06-sandbox-and-v1.md`.

### `C3` - the schedule cannot be reordered, or it grows by two -- RESOLVED

The approved engine design says passes are fixed at compile time with no API to add, remove or
reorder one, and describes **eight**. `SL6` (user ruling, 2026-08-30) grew it to **ten**.

**Ten is operative and already built** -- `kSubstepPassCount = 10` in
`spade/engine/physics/schedule.hpp:282`, verified 2026-09-07.

The subtle part is that `D11`'s parity spine was certified against the eight. That certification
survives because **both new slots are inert with no behavior registry attached** -- which is every
golden-corpus scenario, so the corpus re-proves the inertness on every run rather than it having
been a one-time observation. The amendment nonetheless lands on a claim that had already been used
as evidence, which is worth knowing.

### `C4` - `D10`'s endpoint names Vulkan; S7a shipped a CPU rasterizer -- RESOLVED by what shipped

`D10` and the approved S7 exit proof both name a **Vulkan** renderer as the thing that replaces v1.
`RS1` splits S7 and declares that same exit proof met by S7a, which ships a CPU rasterizer.

**`RS1` is operative, because it describes what exists.** Verified 2026-09-07: `spade/engine/render/`
contains `raster_cpu`, `raymarch`, `scene`, `gltf`, `csg_mesh`, `agreement` -- and **no Vulkan render
backend**. Vulkan in this engine is compute-only (`engine/compute/vulkan/`). S7a merged at `9778260`.

The approved text is simply out of date, and the rule this estate should apply is the plain one:
**where the code shipped, the document that describes what shipped is the operative one.** A draft
that matches reality outranks an approved page that does not.

What remains is paperwork, not ambiguity: `RS14` books the `D10` amendment as *owed* rather than
*made*, and `RS` is unsigned. That is a process debt for the user, not an open technical question.

### `C5` - the scorecard was 7-13 days behind the specs it tracks -- RESOLVED

`superseded/kat-spade-execution-record.md` still modelled `S7` as one undivided phase whose camera-sensor seat
opens with it, and read `D10` as "v1 untouched" without noting that its endpoint had moved twice.

Resolved by consolidation ruling **`CN-6`**: both records merged into `07-status.md`, every status
claim re-verified against git rather than carried forward.

### `C6` - the old Spade repo: frozen archive, or live upstream -- RESOLVED (later user ruling)

- Foundation s.5: "The old standalone repo is now **the archive**... frozen... its origin remote
  remains the historical mirror **until the user retires it**."
- `SL2a` (user, 2026-08-30): "Spade has **its own upstream git repository, unrelated to Kat**,
  which is where v1 lives... Both facts hold at once."

**`SL2a` governs: the external repository is live and unrelated to Kat.** It is the later
statement and it is a direct user ruling; the charter's "frozen archive awaiting retirement" is
simply the older view and is superseded. The charter text stays in `superseded/` as provenance.

The reason this was worth chasing rather than waving through: **`SL14a`'s safety argument depends on
it.** Quarantine-rather-than-delete is justified by "v1's history survives in two places, so nothing
is at risk" -- which under the frozen-archive reading would have had one leg, not two. Under the
operative reading it has both, so `SL14a` stands on the ground it claims. `01-charter.md` carries
the note.

### `C7` - three live `A1`-`A3` namespaces -- RESOLVED

The foundation charter explicitly retired bare `A*n*` citations by renaming its own to `SA1`-`SA3`.
Engine Addendum A then took `A1`-`A11`. But the S1-S4 record defined its own local `A1`-`A3` (demo
rules) **and cited engine Addendum `A3` twice in the same file** -- so inside one document, `A3`
meant two different things.

Resolved by renaming: the demo-scene rules are no longer numbered. They are `07-status.md` s.4
rule 7, "Demo-scene rules". `A1`-`A11` now unambiguously means Addendum A.

### `C8` - ten component types, or nine -- RESOLVED

`SL5` says implementation is limited to "ten"; `SL17` Phase P1 says "the nine component types".

**Both true, stated carelessly.** Ten are *declared*; nine are *functional*; `fluid` arrives with
Plan B. The as-built `component.hpp` is the tiebreaker and has exactly ten (`transform`=0 through
`fluid`=9, `kComponentTypeCount = 10u`). `FluidComponent` is fully wired into the object model --
storage, JSON round trip -- but **nothing consumes it**; there is no SPH pass anywhere in
`spade/engine/physics/`. Marked `RECONCILED` in `04-objects.md`.

### `C9` - SL's phases `P0`-`P7` collide with the charter's pillars `P1`-`P8` -- RESOLVED

"`P1`" meant *object-model phase* in `SL17` and *CPU reference twin* everywhere else in the corpus,
including inside SL itself. There is a third `P` namespace as well: the SPIR-V scan rules `P1`-`P5`.

Resolved by **always qualifying**: prose uses **Plan A / Plan B / Plan C**, the vocabulary already in
use in practice, and where a phase number is unavoidable it is written with its owner attached
(`SL17`'s Phase P5), never bare. Checked 2026-09-07 -- five occurrences remain and all five carry
the qualifier.
 `01-charter.md` keeps `P1`-`P8` as
pillars; `02-engine.md` scopes the scan rules explicitly as "SPIR-V rule `P1`".

### `C10` - the frame pool: engine `render(world, ...)` vs `RS1` "no engine-side frame pool" -- RESOLVED; one bookkeeping gap

**Found during the harvest, not previously known.**

- Engine design s.9 (approved): "The render module (S7) exposes a FramePool: explicit
  `render(world, camera, target)`..."
- `RS1` (draft): "S7a adds **no engine-side frame pool**", and declares the obligation met at the
  C5 seam instead, with the signature `render(scene, camera, target)`.

`RS14`'s amendments-owed table lists engine s.13, s.8/`D7` and `D10`. **It does not list s.9.**

**`RS1` is operative, on the same rule as `C4`: it describes what shipped.** Verified 2026-09-07 --
there is no frame pool anywhere in `spade/engine/render/`; the only `FramePool` in the tree is a
compute-side concept in `engine/compute/vulkan/context.hpp`, which is a different thing. The
obligation is met at the C5 seam, as `RS1` says.

What is genuinely defective is the *bookkeeping*: `RS14` claims to enumerate the amendments this
program owes and it is missing a row. That is a real gap in a draft, and the cheapest moment to fix
it is before sign-off. Recorded in `03-world-and-render.md`.

### `C11` - `SL9f` was in the spec's register but missing from the machine-checked one -- FIXED 2026-09-07

**Found during the harvest; fixed and verified the next day.** See section 4.1.1.

---

## 3 - Redundancy removed

Duplication is what produced most of the contradictions above. Each fact below now has exactly one
home; every other document points at it rather than restating it.

| Fact | Was stated in | Now lives in |
|---|---|---|
| The three determinism grades | 4 places -- foundation `P2`, foundation `SA3` (one screen apart, in the same document), engine parity section, execution-record `D11` row | `01-charter.md` s.4, which also absorbs the band-setting mechanics only the engine copy had |
| The kat-free boundary | 3 places -- foundation s.5 standing rule, `HS2` rules 1-3, `SL1` "where the boundary is enforced today" | `01-charter.md` s.5, naming all three enforcement points. `../integration/spade-c5-adapter.md` keeps only the C5-facing half |
| v1's disposition path list | 2 places, **with opposite verbs** (`RS10b` delete, `SL14a` quarantine) | `06-sandbox-and-v1.md` -- see `C1` |
| "Kat has never linked v1", same evidence | 2 places -- `RS10b` and `SL2`, both citing `spade-prefix.ps1 -DSPADE_BUILD_V1=OFF` | **`01-charter.md`** (boundary section), once. *(This row said `06` until 2026-09-07; it was wrong.)* |
| The eleven scenes / two populations (8 + 3) | 2 places -- `RS10b` and `SL14b` re-derive the identical split | `06-sandbox-and-v1.md`; SL's version is strictly richer (three equivalence axes) |
| `B10` frame-pool semantics | 2 places -- engine `A2`/s.10 and `HS3`, near word-for-word | `02-engine.md` owns the semantics. **Not fully deduplicated:** `../integration/spade-c5-adapter.md` still carries them in its call-mapping table and again in its verification list. Left because the C5 call table needs the per-call obligation inline to be usable; flagged so the claim is not overstated |
| The S7b deferral list | 2 places -- `RS15` and SL s.12 | `03-world-and-render.md` |
| The component-id freezing rationale | 3 places -- the as-built README, `component.hpp`'s header, and `CHANGELOG.md` | `04-objects.md` states it once; the tracked code keeps its own copy deliberately, because that is where someone about to renumber will be standing |

---

## 4 - Defects found by the consolidation

### 4.1 The v1 transfer register was missing a row, and its guard could not see that -- FIXED

**FIXED 2026-09-07, mutation-tested, 818/818 green.** Kept in full because the shape recurs and
because the fix changed which artifact is normative.

The 24th spec's `SL7` register table has **fourteen** rows. The tracked
`spade/docs/v1-transfer-register.md` has **thirteen**. The missing one:

```
| Barycentric wireframe | Barycentric.geom | SL9f -- retire with reason.
  A geometry-shader technique for GPU wireframe overlay; the CPU rasterizer
  draws wireframe directly and needs no equivalent. |
```

It is not folded into the existing "Wireframe / colour render" row -- that row covers
`RenderWireframe`/`RenderColor` and is dispositioned `transferred`, whereas `SL9f` is a distinct v1
surface dispositioned `retired-with-reason`.

**Why the guard did not catch it.** `spade/tests/test_transfer_register.cpp` asserts
`EXPECT_EQ(rows.size(), 13u)`. The count was derived from the file, so the guard pins the file to
itself and agrees with it. It can detect a row *lost after the guard was written*; it can never
detect a row that was never transcribed.

This is `SL7`'s own thesis -- "a silent drop is not a disposition" -- failing on `SL7`'s own
register. It is also the estate's recurring shape: **a completeness check whose expectation is a
hardcoded copy of the thing it checks.**

**The general form** (sharpened by a second session, and it is the clearest statement of this
defect class yet): the guard's own failure message -- *"the register gained or lost a row"* --
already declares its true scope. **It is a CHANGE detector living in a file whose name reads as a
COMPLETENESS check.** An expectation derived from the file catches drift only *after* the
baseline, and is structurally blind to an omission present *at* the baseline -- and the baseline is
precisely the moment a human hand-copied a list out of an authority, which is the moment the
mistake was most likely.

> **A self-derived expectation's blind spot is exactly the original transcription.**

That is why "13 matches the file" was never evidence of anything. Independently confirmed by that
session read-only: `test_transfer_register.cpp:108` pins `13u`; the register body table has 13 rows
at lines 29-41 citing `SL9a`-`SL9e`; `barycentric` appears zero times; the authority enumerates
`SL9a`-`SL9f`.

**What was done.** The row was added as `retired-with-reason`; the pin raised to `14u`; and the
register's header now declares that **it is the authority rather than a transcription.** That last
part is the real fix: the spec that enumerates the systems lives in gitignored `design-specs/`, so it
is absent from a fresh clone and from CI and **no test can ever cross-check against it**. The tracked
artifact is therefore normative and the spec's table is the derived copy. The guard's comment now
states plainly that it is a *change* detector over a human-maintained count.

**Verified, not assumed.** Register parses to 14 rows with SPH still the only `to-transfer`; the
guard was mutation-tested -- removing the row failed `HasEveryRowTheSpecEnumerates` with
"Which is: 13 / Which is: 14" and **only that test failed**, proving the mutation reached the right
guard; full suite **818/818** (759 non-GPU + 59 GPU) green in `spade/build-ninja/release`.

**The "better fix" I first proposed was impossible, and checking that changed the answer.** I had
recommended deriving the expected count from the spec's own table so the guard would compare two
independent sources. It cannot: `.gitignore:161` excludes `design-specs/`, and the test is tracked and
runs in CI on a fresh Linux clone where that file does not exist. Inverting which artifact is
authoritative was the available fix, and it is the better one anyway -- it puts the normative copy
where a clone can see it.

### 4.2 The spec checker went blind on subdirectories -- RESOLVED, and the group is gone

**Outcome:** the defect was real, it was fixed, and then the user ruled the whole `specs` check group
deleted (`9a9fd34b`). Three sessions measured it independently before the ruling: the corpus scan was
non-recursive, so moving specs into realm directories took nine live and seven superseded Spade
documents out of scope while `register_covers_every_spec` went on printing `ok`. It failed **open** --
fewer files scanned reads exactly like an improvement.

Components fixed it on `components/specs-recursion` (`bada8a24`), settling the register-home question
along the way: **realm directories register in their own README.**

**What survived, and it governs this directory.** `core.py::spec_corpus()` remains -- recursive, and
excluding `superseded/` -- because `checks/repo.py::cited_ledgers_exist` uses it and runs in the
`repo` group that the T0 gate, `pr-gate` and `release` all still execute. So:

- This directory *is* scanned. `check repo` -> **0 error** (verified 2026-09-07).
- **`superseded/` is now an interface, not a convention.** The exclusion matches that exact lowercase
  segment and nothing else. Renaming the archive would silently change what a live check reads --
  and one archived document here cites a `.superpowers/` ledger, so the exclusion is what keeps that
  quiet rather than a hypothetical.
- The match is **case-sensitive on a case-insensitive filesystem**; see `08-lessons.md` 2.5.

**The transferable half** (from the session that found it): whatever the register rule turns out to
be, a corpus check must **fail closed** -- notice documents below its scan depth and refuse to print
clean, so the next `mkdir` reports as a problem rather than an improvement.

> **A glob is a scope declaration, and a non-recursive one silently re-declares itself every time
> someone deepens the tree.**

## 5 - Stale text carried forward with a marker

Recorded rather than silently corrected, because these live in approved documents and correcting
an approved spec by editing it is what the estate's own process forbids.

| # | Where | What is stale |
|---|---|---|
| 1 | `01-charter.md` | The foundation's "Kat's engine... spends no energy on standalone packaging or independent releases" -- both halves falsified by `SL1` and `SL2` |
| 2 | `01-charter.md` | `P3`'s "GLFW/GL survive only as long as the port takes" -- `SL14a` moves GLFW/ImGui to the `SPADE_BUILD_SANDBOX` gate, so GLFW outlived the port in a new role. The ruling stands; only the clause is stale |
| 3 | `02-engine.md` | `A9` and `A1` cite `schedule.hpp` line ranges that no longer point at what they named |
| 4 | `02-engine.md` | The approved description of the determinism-replay corpus shape predates the corpus becoming data (S5) |
| 5 | `03-world-and-render.md` | Engine s.10's title "Rendering (S7) -- designed now, built later" is half false; only the S7b residue is unbuilt |
| 6 | `../integration/spade-c5-adapter.md` | The host spec's precedence callout repeats the same words `TS11` withdrew from `F-5` |
| 7 | `../integration/spade-c5-adapter.md` | `HS10`'s "all testing local until the Actions quota resets" was a scheduling constraint, not a verification rule; resolved |
| 8 | `04-objects.md` (note) | `graph.hpp`'s prose still says parent is written as the parent's **name**; the serializer writes **indices** and documents its own departure |

## 6 - Spec vs built

Two places where the 24th spec and the merged implementation differ. The built code wins on *what
exists*; the spec wins on *what was intended*.

| Where | Spec says | Built does | Assessment |
|---|---|---|---|
| `04-objects.md` | `SL4`: attach/detach are "queued, applied only at step boundaries, never mid-step" | `ObjectGraph::attach`/`detach` mutate immediately; there is no queue | **Harmless today**: `Simulation` holds no `ObjectGraph` member -- its only object-model coupling is a borrowed `const BehaviorRegistry*` -- so nothing in a substep can observe a mid-step attach. **Becomes live the moment `SubstepContext` gains an `ObjectGraph`** |
| `04-objects.md` | `SL4`: a graph "serializes by component type **id**" | The serializer writes the registered **name** | Built form is stronger and still satisfies `SL4`'s real requirement (registered, not positional), and it keeps the name-to-id mapping single-sourced |

---

## 7 - What was dropped

Nothing normative. Four sections of the foundation charter were deliberately not carried, all
execution state already superseded:

- s.6 milestone placement / M1b sequencing -- the one load-bearing sentence survives inside `SA1`.
- s.7 the 2026-08-06 gap register (12 rows) -- already annotated as superseded by `A11`.
- s.8 the nine items owed by the engine design session -- discharged by the 9th spec, 2026-08-08.
- s.9 survey vitals and the dead-code adjudication list -- executed during S1.

From the S1-S4 record: the narrative prose was dropped, but its forward-obligation table and its
per-task commit traceability are carried in full into `07-status.md`, and its one durable lesson
(the libm parity break) is the closing section of `07-status.md` and section 1.1 of
`08-lessons.md`.

---

## 8 - What still needs the user

| # | Item | Why it cannot be settled here |
|---|---|---|
| 1 | **`RS1`-`RS15` sign-off** | Process debt only. The technical content is settled -- `C4` and `C10` are resolved by what shipped. `RS14` should gain its missing engine-s.9 row before signing |
| 2 | **`SL1`-`SL18` sign-off** | Process debt only. `C3`'s schedule growth is built and shipping |
| ~~3~~ | ~~**4.1 / `C11`** -- the missing `SL9f` register row~~ | **CLOSED 2026-09-07.** Row added, pin raised to `14u`, register made normative, mutation-tested, 818/818 green. **Uncommitted** -- see below |
| ~~6~~ | ~~**4.2** -- `specs.py` scope narrowing~~ | **CLOSED 2026-09-07.** Fixed by Components (`bada8a24`), then the whole `specs` group removed by user ruling (`9a9fd34b`). `spec_corpus()` survives, recursive, excluding `superseded/` |
