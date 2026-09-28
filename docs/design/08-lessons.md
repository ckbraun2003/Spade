# Spade -- engineering lessons

**What this document is.** The durable, Spade-specific engineering knowledge earned during
S1-S7a and Plan A. Each entry states the *mechanism*, not just the rule, because a rule whose
mechanism you have forgotten is a rule you will talk yourself out of.

**Scope.** Engine lessons only -- determinism, parity, guards, and this box. General process
lessons (how to plan, how to review, how to work across sessions) live in `tasks/lessons.md`,
which is per-checkout and does not travel with a branch.

> **These are not tracked.** `design-specs/` is gitignored, so nothing here survives a fresh
> clone or reaches a session on another machine. The lessons that must not be lost have been
> pushed into places that *are* tracked: standing rules into the specs, mechanisms into
> `spade/engine/**/README.md` and code comments, and enforcement into tests. Where an entry
> below names such a home, that home is the real one and this is the index.

---

## 1 - Determinism and bit-portability

### 1.1 libm transcendentals are not bit-portable, and init-time draws are the least forgiving consumers

**Mechanism.** IEEE 754 mandates correct rounding for `+ - * / sqrt` only. Everything else --
`sinf`, `cosf`, `expf`, `logf` -- is implementation-defined, and MSVC's CRT and glibc genuinely
differ by 1 ulp at specific arguments.

**Why it nearly escaped.** Four golden scenarios were re-baked on Linux; exactly one failed, at
tick 0, before any physics ran. The two divergent arguments were consumed by Box-Muller during
Dryden initialization. **An init-time draw lands in registered state unattenuated, while
per-substep draws pass through a ~7e-3 noise gain that rounds a 1-ulp difference away.** So three
scenarios passed *by luck* -- and a suite that passes by luck is worse than one that fails,
because it certifies the wrong thing.

**The guard.** `core/fp32_math` -- `log32`/`sin32`/`cos32`/`exp32` built from mandated ops only,
exhaustively verified to <=1 ulp over all 2^24 inputs. A comment-aware source-scan canary rejects
any libm transcendental on a path feeding registered state or a digest, with a pattern set
covering both C spellings and glm's wrappers.

**What was *not* done, deliberately:** no tolerance loosened, no per-platform goldens, no digest
weakened. A Linux-only divergence is a bit-portability defect, never a tolerance adjustment.

### 1.2 The GPU has the same problem, one layer down

Vendor transcendental intrinsics are the GPU's libm. Worse, the box's integrated GPU **flushes
denormals by default**, and `slangc` cannot decorate a sum-of-products, so accumulation order in
an `OpDot` is unspecified.

The mechanism is enforced by scanning the compiled SPIR-V rather than trusting the source:
`P1` NoContraction - `P2` no `OpDot`/sum-of-products - `P3` denorm-preserve declared - `P4` no
Int64 capability - `P5` no GLSL.std.450 exp/log/trig - `E1`/`E2` no `OpFDiv`/`sqrt` in fp32_math
modules. Asserted per compiled variant, with a coverage count derived from the build so a new
module cannot slip past unscanned.

**Corollary that is easy to lose:** *GPU results are never a golden source.* Parity compares two
live runs. A golden generated on a GPU would bake one device's rounding into the contract.

### 1.3 Reductions must be order-invariant by construction, not by luck

Float addition is not associative, so any workgroup-shared reduction whose order depends on local
size is a parity bug waiting for someone to change `workgroup_size`. The rule is structural: per-item
writes, or fixed-order serial folds -- per-world serial wherever the CPU is serial per world.

`workgroup_size` is then a **live** knob ({32, 64, 128}) with per-size compiled variants, and the
invariance sweep is **mutation-killed** -- a deliberately local-size-dependent reduction was
introduced to prove the sweep can actually tell the difference. A sweep that has never been shown
to fail is not evidence.

### 1.4 A single-process determinism test cannot see indeterminate memory

Two runs in one process read the *same* uninitialized bytes and agree perfectly. The comparison is
structurally blind to the entire class of defect it looks like it covers. This is recorded in the
header of the test that has the limitation, not in a separate document, because that is the only
place a reader will be standing when it matters.

---

## 2 - Guards that cannot fail

This is the estate's dominant defect family. Every instance passed review, and several passed for
weeks, because a guard that cannot fail looks exactly like a guard that never fails.

### 2.1 One invariant, one site

**Mechanism.** Two sites maintaining the same invariant make each other untestable: neither can be
removed without the other silently covering, so no mutation reaches either one.

**Why it gets written.** Not sloppiness -- *diligence*. Belt-and-braces defensive coding is exactly
how it appears, which is why review waves it through.

**The rule.** A recycled slot is reset in `create()` and nowhere else; `destroy()` bumps the
generation and frees the slot, and scrubs nothing. Place the invariant at the site that
*establishes* the guarantee, and only there. (See `spade/engine/objects/README.md`.)

### 2.2 A second transcription is not a copy -- it is a fork

The Vulkan step recorder held its own transcription of the substep schedule with hardcoded pass
indices and a comment claiming it mirrored the original. **The schedule grew from eight passes to
ten with every GPU test green**, because nothing ever compared the copy back to its authority.

**The demonstrating mutation:** change the thing *and* dutifully update its test. If only a
cross-check catches it, that cross-check is the entire guard -- and if there is no cross-check,
there is no guard.

Now pinned by `Schedule.RemovingTheBehaviorSlotsLeavesSpecSectionThreeExactly`.

### 2.3 A mutation must be verified to falsify the thing you meant

A reordering mutation coincidentally produced exactly the sequence a *different* order test
expected, so it turned something red without testing what it claimed to. Check what the mutation
actually falsified, not merely that something failed.

### 2.4 Derive a guard's input set from an independent source

A guard whose search alphabet comes from its own allowlist can only find what it already permits;
an empty allowlist searches for nothing and reports clean.

**The sharpest form of this, and worth memorising:** a check whose expected value was read off the
thing it checks is a **change detector**, not a **completeness check** -- however its filename
reads. It catches drift after the baseline and is structurally blind to an omission present at the
baseline. And the baseline is exactly when someone hand-copied a list out of an authority, which is
when the mistake was most likely.

> **A self-derived expectation's blind spot is precisely the original transcription.**

Live instance: `spade/docs/v1-transfer-register.md` has 13 rows against the 24th spec's 14, and
`test_transfer_register.cpp` asserts `EXPECT_EQ(rows.size(), 13u)` -- so the guard sides with the
file over the authority. Its own message, "the register gained or lost a row", already says it is a
change detector. The fix is to derive the expected count from the authority, so the check compares
two independent sources instead of one source with itself.

### 2.5 A guard's scope can be a string, and strings have case

`core.py::spec_corpus()` excludes archived specs with `"superseded" not in p.relative_to(specs).parts`.
On NTFS that is a guard whose failure mode is silence on a filesystem that cannot see the difference:

| directory | in corpus? |
|---|---|
| `superseded/` | excluded -- correct |
| `Superseded/` | **SCANNED, silently** |
| `SUPERSEDED/` | **SCANNED, silently** |
| `archive/`, `old/` | scanned |

**Rank failure modes by whether inspection can see them, not by likelihood.** `archive/` looks wrong
to a reviewer. `Superseded/` looks *right*, resolves identically for every path operation, and renders
identically in Explorer and `ls`. The rarer failure wins because nothing downstream catches it either.

**The condition is narrower than it first appears, and the narrowing is the dangerous part.** The trap
springs only when a realm creates its *first* archive with the wrong case. Once `superseded/` exists,
NTFS resolves every variant to it and `parts` yields the existing lowercase name. So a probe run in a
realm that already has an archive **reports a false all-clear** -- one session's `mkdir Superseded`
did not create anything, it opened the existing archive and dropped the probe file among eleven
retired documents. The corpus count did not move, which read as "excluded"; the only thing that caught
it was the cleanup `rmdir` failing with "directory is not empty".

*(Found by SDK, condition narrowed and measured by Editor, verified here in `core.py`.)*

### 2.6 Do not transcribe a condition under a different name than its source

Verifying the above, I copied the source's predicate -- `"superseded" not in parts` -- and printed it
under a column headed `excluded=`. It is a **keep** predicate, so my output said the exact opposite of
the truth, and I only caught it reading the numbers.

The session that got it right had computed the label's own condition (`"superseded" in parts` under
`excluded=`) rather than reusing the source's expression. Their label and their expression were the
same claim, so there was no gap for an inversion to live in.

> **Transcribing a condition under a different name creates a second claim that nothing checks against
> the first.** Either compute what your label asserts, or label it with the source's own words.

### 2.7 "No check fails" is not "nothing is broken"

My own round-1 research listed six tracked files whose citations would go stale when I moved the
Spade specs -- `spade/README.md`, `math_ops.hpp`, `baselines.json`, and three others. I wrote the
list, moved the files, and repointed none of them, because no check covered those citations at the
time. A check was written the next day and immediately found one; grepping for the rest found two
more the new check structurally cannot see (`.hpp` is not in its extension list, and its `/tests/`
exemption swallows real provenance records in `bench/`).

**I had the defect list in front of me and treated a silent tool as a verdict.** The inversion is
the one this whole document is about, and having catalogued it seven times did not stop me making
it. Absence of a red result is evidence only in proportion to what the check actually scans.

### 2.8 Absence is invisible to a resolution check

A realm finished its consolidation, ran a link checker and a decision census over its own directory,
and got clean results -- while the estate index had no row for it at all. **An index row is not a
link.** Nothing was broken; something was *missing*, and a checker that resolves what exists cannot
see what was never written. Same shape as the non-recursive glob, one level up: the check was correct
and its scope silently excluded the defect.

*(Editor's, recorded because the shape generalises past indexes.)*

### 2.9 Guard the authority, not only the consumers

A check placed over every consumer of a contract leaves the *producer* unverified. Go to the other
side of the contract and guard it there too, or the authority can drift while every consumer
faithfully agrees with it.

### 2.10 A `static_assert` pins layout, never marshalling

Layout guards say the fields sit where you think. They say nothing about whether the code writing
into them writes the *right* field. A transposition in a 16-element mapping survived four pinned
sites and was caught by one case out of 973.

**And:** a fixture that documents its purpose is not a test of that purpose. Naming a thing is not
asserting it.

---

## 3 - What a result actually describes

### 3.1 A ctest tally describes a build tree, never a commit

Three trees on this box report different totals for the same source with the same flags. Every
number needs `(commit, build directory)` or it is not evidence. The corollary caught a real
mistake: a phase gate quoted `ctest -L T0` when the actual gate is `scripts/test.ps1 -Tier T0`
(pytest + kat-dev + ctest) -- the ctest leg alone is about a third of it.

### 3.2 A grep describes a working tree, never a commit

The same error in a different costume. A header was grepped, a value found, and reported as a
property of committed master -- when it was an **uncommitted** file in a worktree a peer was
editing. Attribute a fact to a commit only through `git show HEAD:<path>`.

**A third dimension: a shared working tree tells you WHAT changed, never WHO.** Five sessions work
this checkout; `git status` printing ` M tools/kat_dev/checks/specs.py` carries no authorship at all.
Two sessions read that line and attributed an edit to me because I was the one they had been
corresponding with about that file. The cost was not a wrong belief but **paralysis** -- three
sessions deferring to an owner who did not exist.

What resolves it is two commands, and the second is the point:

```
git log --all -S "<a distinctive string from the change>" -- <path>   # find the commit
git branch -a --contains <commit>                                     # READ THE REF
```

**Read the ref, never the author.** `git log --all --format='%an'` returns one human across every
session here. And check both layers: the branch may hold the work while the tree holds *more* than
the branch. *(Fuller account of the multi-session etiquette in `tasks/lessons.md`; what belongs here
is the verification technique.)*

### 3.3 A swallowed error is indistinguishable from an empty result -- and empty reads as refutation

A peer set out to verify a defect I had reported. They grepped the spec by the filename I had
given them -- but I had moved that file into `superseded/` an hour earlier and had not said so. The
command carried `2>/dev/null`, so the *no such file* error was discarded and **an ERROR arrived
looking like an EMPTY RESULT**. Read literally, that said the spec contains no such ruling and my
report was false. They were one sentence from telling their user exactly that.

**Why this one is nastier than its cousins.** A vacuous *guard* fails toward silence. A vacuous
*verification* fails toward **confidently contradicting the person who actually measured** -- so it
does not merely miss a defect, it manufactures a refutation and hands it to a human.

Two rules, both cheap:

- **Never `2>/dev/null` a search whose emptiness you intend to treat as a finding.** If absence is
  the evidence, the error channel is part of the evidence.
- **Grep the realm, not the remembered filename.** A path in a command is a scope declaration
  exactly like a glob is, and a stale one turns a check into a vacuum.

And the half that was mine: **when you report a defect by citing a document, cite where it is
now.** I had moved 42 references' worth of files and left every citation bare. Checking that the
files still existed was the wrong verification -- the risk was never that the target was missing,
it was that a reader would look in the old place and find nothing.

### 3.4 A conclusion written before its result is not a check

I ran `git diff <commit> -- tools/ --stat` to decide whether a peer's pending deletion would destroy
unsaved work, and paired it with a hardcoded label: *"empty stat above = nothing would be lost."*

**The stat was not empty.** The working tree held an entire uncommitted fix on top of that commit.
Had I trusted my own label instead of reading the output, I would have handed a false all-clear to a
session that was holding a destructive action *specifically* on my answer.

The mechanism is the day's recurring one, at its purest: **the label asserted the finding, so the
command could only ever confirm it.** A check whose conclusion is authored before its result has the
same epistemic content as no check.

It generalises past shell echoes. Any assertion that names its own expected outcome -- a test
comment saying what the assert proves, a commit message written before the run, a status line
drafted from the plan rather than the output -- carries this risk, and carries it most when you are
in a hurry, which is exactly when someone is waiting on the answer.

### 3.5 "The merge succeeded" is not "the work landed"

Ask `git merge-base --is-ancestor`. And to prove the shipped tree is the tested tree, ask that
`git diff <branch> master` is zero lines. A zero-conflict merge can still break the build, because
git conflicts on *text* while a build breaks on *meaning* -- a cross-file signature change
conflicts with nothing.

### 3.6 Local-green is a claim about one toolchain

The first hosted CI run after a quota outage found six independent failures, three of them
invisible to a Windows/MSVC-only battery.

---

### 3.7 A measurement can return the right number for the wrong reason

Checking whether these documents were CRLF or LF, I ran `grep -c $'\r' file` beside `wc -l` and
read the two as agreeing: *"lines=468 cr=468, therefore uniformly CRLF."* Every file in the
directory agreed. **The files are uniformly LF.** In this shell `$'\r'` did not reach grep as a
carriage return, so the pattern was empty and `grep -c` counted *every line* -- it was returning
the line count a second time, under a different heading.

What made it survive scrutiny is that the wrong method produced exactly the value the right method
would have produced **if the hypothesis were true**. Equality between the two columns was read as
corroboration when the second column was a copy of the first.

> **Two measurements agree trivially when one of them is the other.** Before reading agreement as
> evidence, check that the two could have disagreed. The method itself was never validated: run it
> once on a file whose endings you already know. These files are pure LF, so a working check reports
> `0` and the broken one reports the line count -- **the discriminating case was the one in front of
> me, and I read its wrong answer as confirmation** because I had no correct answer to compare it
> to.

Same family as `3.4` (a conclusion written before its result) and `2.4` (derive the input set from
an independent source): the failure is never the arithmetic, it is that nothing in the setup could
have produced a different answer.

### 3.7a How to settle it when two careful sessions disagree about a number

The same `$'\r'` bug bit a second realm the same day: their `grep -c` reported a file as 315-of-315
CRLF; my byte-count reported 0. **Both of us were careful, both had a plausible instrument, and
neither could tell from the field data which one was lying.** They proposed an explanation --
*"`git show` smudges its output"* -- that was wrong, and recording it would have been worse than
recording nothing: it sends the next reader to a workaround for a problem that does not exist while
leaving the real one live.

What settled it was not a third field measurement. It was a **probe against known ground truth**: a
file built by hand with exactly one CRLF line and one LF line, where the correct answer could be
stated *in advance*.

    grep -c $'\r$'  ->  2    (should be 1)
    grep -c $'\r'   ->  2    (should be 1)

Both match every line. The pattern is empty; `grep -c` returns the line count. One command, and the
question stopped being *"which number is right"* and became *"this instrument cannot count carriage
returns."*

> **When two careful measurements disagree, stop measuring the thing you are unsure about and
> measure the INSTRUMENT on a case whose answer you already know.** A field measurement can only
> ever produce a third number to argue about. A probe with known ground truth is decisive, because
> being wrong about it is impossible to explain away.

This is the resolution procedure for everything in this section. `3.7` says a method must be able to
disagree with your hypothesis; **this says how to find out whether it can** -- and it is cheap enough
that there is no excuse for arguing instead. Credit to the runtime/overseer realm, who ran it on
themselves after I disputed their diagnosis rather than defending the number.

**Practical, for this repo:** `$'\r'` is not a portable way to find carriage returns in this shell.
Use `git ls-files --eol`, which reports index and worktree separately -- the actual question -- and
which a quoting accident cannot defeat. Count bytes in Python for anything it does not cover.

### 3.8 A hand-maintained index of a growing set decays silently

This directory's README carried *"22 annotation blocks"* over groups summing to 21, against 25
blocks actually present. Two stated numbers, disagreeing with each other and with the files. The
index had been correct when written; four markers were added afterwards by edits that did not think
of themselves as index changes.

Nothing could catch it, because **a plausible count is indistinguishable from a correct one**, and
`design-specs/` is gitignored so no test may read it. The fix is not a better number -- it is to
regenerate the list from the files and say so in the section, so the next reader knows whether they
are looking at a derivation or a memory.

> Applies to every count in a spec: ruling totals, document counts, test tallies, register rows. If
> the number is not recomputed, date it, or state what produced it.

---

### 3.9 Verifying every fact in a claim is not verifying the claim

The runtime realm reported that C5's semver had fallen behind its surface: `KATHOST_ABI_MINOR` is
`0`, yet `B2`'s `kathost_caps_query` sits in the header tagged `/* B2 */` -- a shipped additive call
that never moved the minor. I did not take it on trust. I opened the header, confirmed the constant
at `:38`, confirmed the call at `:158`, confirmed the tag. **Three facts, all true.** I wrote the
finding into three documents.

**The conclusion was false.** `git log -S` returns the *same single commit* for the constant and the
call (`eebc4373`, 2026-08-11): they were authored together. The `/* B2 */` tag records **which
amendment specified the call, not when it entered the file.** Nothing additive ever shipped
unversioned; a host reporting C5 1.0 does have capability discovery.

What I actually verified was the **premises**. The **inference joining them** -- *tagged with a
later amendment, therefore added later* -- I inherited whole and never examined. It was never in the
header; it was in my reading of a comment.

> **An annotation is a claim about history, and reading it is not checking it.** `/* B2 */` is
> provenance metadata. The tool that adjudicates provenance is `git log -S`, one command, and I had
> used it earlier the same session to settle a different attribution question. Having the right tool
> and a recent habit of using it did not fire, because the claim did not *feel* like an attribution
> question -- it felt like a version question.

Two things generalize:

- **When someone hands you facts plus a conclusion, the facts are the cheap half.** Verifying them
  feels like diligence and produces the sensation of having checked. The error, when there is one,
  is almost always in the joint -- and the joint is the part with no line number to open.
- **A false finding in a spec is not inert.** *"C5 has fallen into the version hole"* is a sentence
  that sends the next reader to amend a byte-hash-pinned header. Specs are read as instructions, so
  the cost of a wrong one is not embarrassment, it is someone acting on it.

The corrected finding is the one that survives: **no guard ties either header's version constant to
its surface.** C5's minor has correctly never moved and C2's `0->1` for `B27` was correctly done --
both *by attention, not by construction*. That is a missing guard, not a defect, and it is the same
shape as section 6's summary: nothing in the setup could have produced a different answer.

---

### 3.10 A clean check needs its coverage beside it -- and in a spec, "missing" is often correct

Prompted by the runtime realm measuring its own section-pointer check at **12%** coverage while it
printed a clean *"0 broken"*, I measured mine. My reference check matched **13 of the 120** path-shaped
references in these ten documents -- **11%** -- and had printed *"0 missing"* every time I ran it,
including in the verification paragraph of a report to the user.

> **"0 broken" over 11% and over 100% print identically.** A clean result is meaningful only in
> proportion to what was scanned, so the scan size belongs in the output, not in the author's memory.

Widening it to the whole population surfaced 19 candidates. **Exactly two were real:**

- `../kat-runtime-sdk-contracts.html` -- the runtime realm's consolidation moved it under
  `runtime/superseded/` while my link still pointed at the flat path. **This class will recur**: every
  peer realm consolidating moves its originals, and my links rot when *they* land, not when I edit.
- a `spade/engine/tests/...` path that never existed (`git log --diff-filter=A` returns nothing; the
  real file is `spade/tests/...`).

**The other seventeen were the checker being wrong, and one of them nearly cost a true statement.**

`RS1a` says `dronesim/spade/raster.cpp` and `raster.h` are **deleted**. The file does not exist, so the
check flagged it -- and the flag was the *proof that the sentence is true*. Verified: added in
`837b97e6`, since removed; `dronesim/fake/raster.cpp` is a different, live file that eight sources
still include. Had I "fixed" the path to the one that resolves, I would have turned a correct
statement into a false one, in the direction that reads as tidying.

> **A reference checker cannot distinguish a broken link from a deliberately recorded absence** -- and
> this directory is full of the latter: `RS1a`'s deleted raster, `SL9f`'s retired `Barycentric.geom`,
> `SL14a`'s unexecuted quarantine. In a document whose job includes recording what was removed,
> **non-existence is frequently the correct state**, and a checker that treats it as a defect will
> push the text away from the truth every time it is obeyed.

Resolve each candidate against history (`git log --diff-filter=A -- <path>`) before believing it. Same
root as `3.9`: the tool reported a true fact (*no file there*) and I supplied the false inference
(*therefore wrong*).

**Then check the other direction, because it is the one you can break.** Outbound refs rot when peers
land; **inbound refs -- what other realms cite into yours -- rot when you edit.** Having cut section 1
of `../integration/spade-c5-adapter.md` the same day, I checked: 5 distinct targets cited by peers, all resolve, and both
section-level citations (`05` s.3.1 `G2`, s.10 `HS9`) still exist and still say what the citing
document expects. The cut had stayed inside `1`/`1.1` and shifted no heading numbers. **A section
number is an interface**, and editing within a section is safe in a way that editing *around* one is
not.

**A postscript worth more than the result.** The runtime realm had just warned me, in the message that
prompted this, that resolving realm-relative and repo-relative paths against a single base produces
false BROKENs -- a bug the editor realm found first and they then rediscovered independently. I wrote
the inbound checker minutes later and **single-based it**, flagging `spade/engine/core/rng.hpp` and
`spade/tests/bench/` as broken when both are repo source paths that resolve fine. My *outbound*
checker, written an hour earlier, already tried three bases.

> **A warning received is not a defence.** The knowledge was current, correct, and mine -- it simply
> did not transfer to the next artifact I built, because I was not looking for that failure in *this*
> tool. Defects do not propagate through understanding; they propagate through the code you write
> next. The only thing that actually caught it was resolving the two flags before reporting them.

---

## 4 - This box

| Symptom | Cause | Response |
|---|---|---|
| A suite dies partway through with no error | **Backgrounded builds and test runs get killed on this box.** | Foreground only. Partition long runs into chunks that provably cover the whole suite, and verify the partition with `-N` counts. |
| A revert "does not fix" the thing it should | `Copy-Item` preserves `LastWriteTime`, so ninja sees no work to do and you grade the *previous* binary | Use `cp`/`touch` after restoring. "ninja: no work to do" can lie about test binaries. |
| Nondeterminism across two sessions in the same window | **A full disk presents as a determinism bug.** It also truncated a source file to zero bytes mid-write | Run `df` *before* debugging determinism. |
| A `.ps1` misbehaves | Non-ASCII characters in PowerShell files | `.ps1` on this box must be ASCII-only. |
| PowerShell logs are unreadable | `*>` redirection writes UTF-16 | Expect it, decode accordingly. |
| A predicate is always false | `if (git merge-base --is-ancestor A B)` in PowerShell tests **stdout**, not the exit code | Use bash for exit-code predicates. |

**Never invoke a compiled test executable directly.** Use `spade/scripts/build.ps1` then
`test.ps1`, or ctest. Direct invocation bypasses the working-directory and environment contract
the suite is registered with.

---

### 4.x A guarded edit fails loudly; an unguarded write succeeds silently

`tasks/` is untracked **and shared by every session in this checkout**, while the user's CLAUDE.md
instructs each of them to *"write plan to `tasks/todo.md`"*. Six sessions, one path. On 2026-09-08 a
peer wrote their plan there with `cat > tasks/todo.md` and my 658-line plan ceased to exist. No
fault of theirs -- they followed the instruction, as had I.

**Untracked shared state is the worst case, because every recovery mechanism is a git mechanism.**
No index, no HEAD, no reflog, no conflict, no stash. The rules that protect this checkout's
*commits* -- private `GIT_INDEX_FILE`, never `git stash`, verify per-parent -- all stop at the
tracked boundary, and the file we were each told to plan in sits outside it.

> **The asymmetry that decides whether you lose minutes or lose work** (named by the peer who hit
> it): `Edit` against a unique anchor **fails loudly** on a file that changed under you.
> `Write` and `cat >` **succeed silently**. Same intent, opposite failure mode.

I found the loss only because a guarded script asserted its anchor was present and it was not. An
unguarded `Write` would have overwritten *their* plan in turn and neither of us would have known.
**Prefer anchored edits over whole-file writes on anything another session can touch** -- not for
elegance, but because the assertion is the only thing that reports a concurrent change at all.

Practical: per-session filenames (`tasks/todo-<realm>.md`), and append rather than rewrite for
shared accumulating files like `tasks/lessons.md`. The root cause is a line in the user's own
CLAUDE.md naming one path to everybody; **that is theirs to change, not a session's** -- surfaced,
not edited.

**The part that makes this more than a tooling note.** Another realm hit the same hazard at **03:18
the same day**, mitigated it, and wrote the reason into the header of *their own* plan file --
eleven hours before it destroyed mine, in a document no other session had cause to open. **Nobody
was ignorant; the knowledge was simply stored where only its author would find it.**

> **A warning recorded is not a warning delivered.** The estate's own CLAUDE.md already says it:
> anything that must reach another session belongs in a tracked artifact -- a failing test, a
> comment beside the code, a `docs/` page, a commit message. A mitigation written into an
> untracked, session-local file protects exactly one session, and reads afterwards like diligence.

Pair it with `3.10`'s postscript, which is its mirror: **a warning received is not a defence** (I
wrote the single-base resolver bug with the warning open). One says knowing does not protect the
next thing you build; this one says knowing does not protect anyone else at all. Between them they
account for most of what went wrong across three realms in one day.

### 4.y The shared git index is shared state, and its failure is DEFERRED

2026-09-08. A commit titled *"docs(testing): the ui-tree label counts were off by one"* carried
**536 deletions** that removed the base-package world, its recipe, its scene, and the W6c benchmark
from the tree. Nothing was deleted from disk; the files sat there untracked the whole time. **Only
the tree forgot them.**

**The mechanism, which is a hazard in the mitigation rather than in anyone's judgement.** Build an
index from a `git read-tree` base that predates a peer's commit, then commit it, and git faithfully
records that peer's files as **deleted** -- the tree genuinely lacks them. The working directory is
never touched. Nothing looks wrong locally. The deletions land under a commit message about
something else entirely.

**Every check we had passes on the resulting tree:**

| check | verdict on a tree that lost your files |
|---|---|
| `git show <mine> --stat` | **passes** -- your tree was right |
| per-parent diffing | **passes** -- shows what *you* changed, never what a later commit did |
| the local test suite | **passes** -- the files are on disk |
| `git status` | shows `??`, which reads as *new work in progress*, not *master lost these* |
| **`git worktree add --detach` at HEAD + run the suite there** | **the only one that fails** |

> **Verifying your work landed is not verifying your commit.** Per-parent diffing answers *"what
> did I change"*; this failure lives entirely in *"what happened to it afterwards"*. Two different
> questions, and we only had a tool for the first. The second costs ~40 seconds.

**It compounds while everyone behaves correctly.** Every correct commit advances HEAD past a stale
seed, so the blast radius of an index nobody is touching grows on its own. Ours reached **558
staged deletions** before it was cleared, and by then it would also have reverted the fix for its
own first casualty.

**A stale index is indistinguishable from a fresh one by inspection** -- `git status` against it
looks entirely normal, because from the index's point of view the tree merely has untracked files
and modifications. Three defences, strongest first:

1. **`git commit <pathspec>`.** A stale index cannot express deletions of files you never named.
   It fails safe instead of requiring discipline.
2. **`git diff --cached --stat HEAD` before any commit.** One second, and the only thing that
   distinguishes stale from fresh.
3. **A fresh worktree at HEAD as the closing step.** The only check that catches a tree already
   broken by someone else.

And if you seed a private index: **`read-tree` the HEAD you are about to commit against, captured
in the same operation** (`PARENT=$(git rev-parse HEAD); git read-tree "$PARENT"`). The private-index
rule without that refinement reintroduces the hazard it exists to prevent.

**THE ROOT CAUSE, found last and explaining all of it.** `commit-tree` + `update-ref` move **HEAD**
and never touch the **shared index**, which is still at the pre-commit tree. So the moment your
commit lands, your own file differs from HEAD *in the shared index, in reverse* -- it presents as
**staged to revert your own commit**, and the next bare `git commit` by anyone takes it. **Nobody
staged anything.** The private-index recipe produces this as a side effect every single time, and
each commit adds its own files, which is exactly why the index re-armed within minutes of every
clear and grew from one file to seventy-six across a day.

The missing step, path-scoped so a peer's genuine staging is untouched:

    git reset -q -- <the paths you just committed>      # after update-ref

> **A mitigation with an unstated cleanup step is a slow leak.** Everyone following the rule
> correctly was manufacturing the hazard the rule exists to prevent -- and because the artifact is
> *coherent* (a valid earlier version of your file), nothing complained. Same property as `4.y`
> above: the damage looked exactly like a legitimate state.

**Forensic note worth keeping.** Diffing a suspect index against *every commit in range* identifies
its base exactly: ours matched one commit with **zero** files differing, which proved it held no
work-in-progress and made clearing it safe. *"Differs from a committed tree in zero files"* is the
difference between a loaded hazard and someone's unfinished work, and it is one command.

**And the property that decides whether you find out at all.** A peer later lost an uncommitted
file to a path-scoped revert (`git checkout -- <path>` / `git restore` / `git clean`, the same
destructive class as `git stash` and likelier, because a session tidying "its own" file cannot know
a peer edited it). They caught it in minutes -- but only because the revert was **partial**: a
header at one schema against fixtures at another is an incoherent tree, and incoherence is loud.

> **Partial damage announces itself; complete damage does not.** A destructive operation is
> dangerous in proportion to how **coherent** it leaves the tree. Damage that breaks an invariant
> is caught by whatever checks that invariant. Damage that restores a consistent *earlier* state is
> invisible to every check you own -- nothing is wrong with the state, only with the fact that it is
> not the state you were in.

This is why the stale index above was the dangerous one: a tree missing four of my files was
perfectly self-consistent, simply an earlier valid repository. It is also why `b324092e` was the
loud half of that incident -- my roster entry naming an absent scene was the incoherence the silent
deletion never produced, and it is what made the loss findable.

It explains the check hierarchy exactly: **the test suite tests coherence; the fresh worktree tests
identity against HEAD.** A complete revert passes the first and fails only the second.

## 5 - Planning

### 5.1 Ask what a test would go red for, before writing it

Nine defects were found in one twelve-task plan this way. **None were found by code failing** --
everything built and passed. The implementation was mostly fine; the *description of what was
being verified* was where the rot was.

The most consequential: an unfalsifiable guard specified **twice**, so the plan's own mutation
steps would have printed green and been filed as proof. Others: parent links serialized by a
non-unique key; a `string_view` into a `vector<string>` that reallocates; three pieces of wiring
no task owned; a falsification step that could not falsify; a register row wider than the task
closing it; and one RNG stream shared between two things that must not perturb each other.

### 5.2 A domain-separated stream is not a convenience

Ambient RNG (`std::random_device`-seeded mt19937, as v1 used) means a capture can never be a
numerical reference. Every draw goes through `splitmix64(scene_seed ^ fnv1a64("<domain>"))`, and
two things that must not perturb each other get *different domain tags*. Sharing one stream makes
asking for a velocity move the positions.

### 5.3 A NaN wrench is invisible to every determinism guard

The trace stays well-formed and the digest stays stable. Determinism proves reproducibility, not
correctness -- they are different claims and only one of them is being measured.

### 5.4 `std::hash` in a digested field is a cross-platform landmine

It is not specified to be stable across implementations, let alone platforms.

---

## 5a - When an instrument is confidently wrong: grep and the split literal

**2026-09-18.** A coordinator relayed a quotation as the words of a spade-host test. Before acting on
it I tried to locate it:

```
git grep "tested the mutation, not the guard"
  tools/tests/test_katsdk_export_surface.py:469,518   <- SDK's guard
  design-specs/sdk/LESSONS.md:36                      <- SDK's lesson text
  in any spade test:                                  NOTHING
```

**I reported the quote as misattributed. I was wrong, and the instrument was the reason.** The phrase
**is** the test's own words, at `sdk/tests/cpp/sensor_wire_test.cpp:409-415` — split across adjacent
C++ string literals:

```cpp
ADD_FAILURE() << "... a mutation that dies only on the fake has tested the mutation, "
                 "not the guard.";
```

⭐⭐⭐ **THE GREP DID NOT FAIL. IT SUCCEEDED AT FINDING THE WRONG COPY.** A contiguous search cannot
see a string broken across literals, and **C++ diagnostics are almost always broken across literals**
because they are long and the line limit is not. **The quotations of them, in prose, are not.**

> ⚠ **When a string is split at its ORIGIN and whole in a QUOTATION of it, grep systematically
> attributes to the QUOTATION — because the origin is the one place it cannot look.**

**AND THE FAILURE MODE IS THE DANGEROUS DIRECTION.** A search that returns **nothing** prompts more
looking. **This one returned HITS**, in real files, with real line numbers — so a failed search wore
the appearance of a successful attribution, and the conclusion drawn from it (*"this is SDK's text,
not the test's"*) was specific, checkable-looking and false.

⭐ **This realm greps more C++ than any other, and this is a standing property of doing so** -- not a
curiosity. **Before concluding a diagnostic string does not exist in a tree: search a distinctive
FRAGMENT that would survive a line break, not the sentence.** A five-word fragment from the middle of
a clause finds both forms; the whole sentence finds only the copy.

⚠ **Counted honestly, this was the fourth time in one session that the instrument was wrong and the
subject was fine** -- a probe pinned to `HEAD` whose control expired, a `grep -c '\$'` that returned
zero against known-positive content twice, an artifact check that looked in the wrong directory, and
this. ⭐⭐ **Three of the four returned ZERO and were caught by having a positive control. This one
returned an ANSWER, and nothing but going back to the source could have caught it.** *A positive
control proves an instrument can detect presence; it does not prove the thing it detected is the
thing you were looking for.*

---

## 6 - The pattern behind all of it

> **A guard validated only against something that cannot vary is not a guard.**

Three independent programs hit this in one month. The training catalog shipped five guards that
passed under the exact mutation they existed to catch. An editor critical survived 471 green tests
because the assertions could not see it. Spade's schedule grew by two passes with every GPU test
green.

The generalization, and the thing worth carrying forward:

> **A passing test is a claim, and claims decay.** Most of the work in a maintenance pass is
> re-establishing what the existing claims still mean -- and the recurring answer is *less than
> they say*.

### What actually worked, measured across one day and three realms

On 2026-09-08 three sessions -- Spade, Runtime and Editor -- produced **four wrong-subject
findings** between them, each stated confidently, each **falsifiable in a single command nobody ran
until afterwards**:

| The finding | What it actually was | The one command |
|---|---|---|
| "C5's semver fell behind its surface" | The tag records which amendment *specified* the call, not when it landed | `git log -S` |
| "these references are broken" | Realm-relative vs repo-relative paths against a single base | resolve against three bases |
| "`RS1a` cites a missing file" | It cites it **because it was deleted** -- the flag was the proof the sentence is true | `git log --diff-filter=A` |
| "`bench_sim.cpp` calls `UseRealTime`" | `grep -c` returned 1 for a **comment saying it does not** | open line 420 |

A fifth, `viewport_widget.cpp:297`, sat in my own grep output while I wrote a finding that needed
it; a peer found it.

> **The only thing that caught these was resolving each hit before reporting it** -- not reading it,
> not classifying it: opening what the hit points at. Every one was one cheap command from being
> killed. The expensive part was never the check; it was believing the check had already happened.

### But a discipline is not a remedy, and the same day produced the better answer

That is something a careful session does, and the day's own evidence is that careful sessions did
not do it. **The editor realm quoted this file's `spread_policy` block to two other sessions and
then built a timing instrument with no minimum-time floor**; I wrote a single-base path resolver
into a new checker an hour after being warned about single-base resolvers, by the session I had
just warned about something else.

> **Knowing the control does not install the control.** Vigilance is the thing that failed, in
> every one of these -- which rules out *"be more careful next time"* as the remedy.

The durable form is to make the instrument carry its own trustworthiness, so the number cannot be
read without it. The editor realm's fix, and the best formulation anyone produced that day:

> *"My bench now prints its own between-batch spread on every line -- not because I'll remember to
> check, but so I can't read the number without seeing whether it's trustworthy."*

The same shape as three other findings here, which is what makes it a general rule rather than a
benchmarking tip:

| The number | What must travel **on the same line** |
|---|---|
| a benchmark timing | its between-batch spread |
| a throughput row | which basis it is on (`cpu_time` vs `real_time` differ ~10x here) |
| a clean checker result | its coverage (mine printed "0 missing" over **11%**) |
| a pinned spec figure | what would make it wrong |

> **A measurement that cannot be read without its own reliability is the only version that survives
> being skimmed** -- and everything in a spec is eventually skimmed.
