# Test/Docs — lessons

What verification here has taught, stated as mechanism plus rule. The full accounts, with dates and incidents, are in `../superseded/2026-09-consolidation/08-lessons.md` (cited as "old §n").

## Guards that cannot fail

The dominant defect family. A guard that cannot fail looks exactly like one that never fails, and it passes review.

- **Show the mutation.** Break the thing on purpose and watch the guard go red, then check that the red came from the thing you broke. A reordering once turned a *different* test red by coincidence (old §2.3).
- **A copy is a fork.** The Vulkan recorder kept its own transcription of the schedule, and the schedule grew from eight passes to ten with every GPU test green. Two copies of one authority need a test that compares them (old §2.2).
- **A self-derived expectation is blind to the original transcription.** The transfer register's row count was read off the register, so the guard agreed with a missing row (old §2.4, now fixed: 14 rows).
- **Two sites, one invariant: neither is testable.** Each covers for the other, so no mutation reaches either (old §2.1, `TD-9`).
- **A `static_assert` pins layout, not marshalling.** A transposed 16-element mapping survived four pinned sites (old §2.10).
- **A skip can substitute the subject.** A test that skips when its real subject is absent reports green about nothing (`TD-5`).

## What a result describes

- **A ctest tally describes a build tree,** and a grep describes a working tree. Neither describes a commit. Attribute to a commit only through `git show <commit>:<path>` (old §3.1–3.2).
- **An error swallowed into an empty result reads as a refutation.** Never `2>/dev/null` a search whose emptiness you will treat as evidence (old §3.3).
- **A label written before the result** turns a check into a confirmation (old §3.4).
- **Two measurements agree trivially when one is a copy of the other.** In this shell `grep -c $'\r'` counts every line. When two careful measurements disagree, probe the instrument on a case with a known answer. For line endings use `git ls-files --eol` (old §3.7–3.7a).
- **A clean check is worth only its coverage.** "0 broken" over 11% prints the same as over 100% (old §3.10).
- **Grep cannot see a string split across C++ literals,** so it attributes the string to whatever prose quotes it. Search a short fragment (old §5a).
- **`--no-tests=error` fires only on zero registered tests.** A battery that always skips is invisible to it. GPU tests skipped on every automated run for weeks while being counted as coverage (07 §4 rule 14).

## Determinism

- **An init-time draw is the least forgiving consumer.** A 1-ulp libm difference passed through per-step noise gain and vanished, but landed whole in state at tick 0. Three of four scenarios passed by luck (old §1.1, `TD-3`).
- **The GPU's libm is its transcendental intrinsics,** and the old box's iGPU flushed denormals unless told not to (old §1.2, SPIR-V rules).
- **A single-process determinism test cannot see uninitialised memory,** and a NaN is deterministic (old §1.4, §5.3).
- **`std::hash` in a digested field** is not stable across implementations (old §5.4).

## Working in a shared checkout

- **A stale index deletes a peer's files under your commit message,** and every local check passes. Commit with explicit paths; prove a tree with a fresh worktree (old §4.y).
- **A guarded edit fails loudly; a whole-file write succeeds silently.** On untracked shared files there is no recovery at all (old §4.x).
- **A warning recorded where only its author looks is not delivered.** Put what others need in a tracked file (old §4.x).
