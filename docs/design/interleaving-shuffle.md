# The interleaving-shuffle mode

> **Design only. `§13.3` is OPEN and the user has not ruled.** Nothing here is built, and nothing
> here should be built until they do. Written because this is the one part of `§13.3` that is
> buildable *regardless* of which way the thread/process question goes — it is worth the same
> whether lanes stay deterministic forever or gain a real-concurrency mode tomorrow.
>
> **Authored by Spade because Spade owns the determinism this would spend and the digest machinery
> it would use. THE IMPLEMENTATION SITE IS NOT SPADE'S** — lanes are scheduled in `runtime/`, and
> this note names that site without reaching into it.

---

## The one-line argument

`§13.3` proposes that **a lane is its own execution context, with no global priority between
lanes**, and the recommendation adopted the model while deferring the mechanism: execute
deterministically, model the budget, keep real concurrency as a mode. That is right. But it leaves
one sentence doing load-bearing work with nothing behind it:

> **"No global priority between lanes"** — while the deterministic default gives every tick a
> **total order**, supplied by the arrangement.

Under real concurrency there is no total order. Under the deterministic default there is one, and
it is stable, reproducible, and *authored*. So the two are **not observationally equivalent**, and a
package written against the deterministic order can silently acquire a dependency on it that the
model never promised.

> ### AN UNSTATED NON-REQUIREMENT BECOMES A REQUIREMENT BY INHERITANCE.

The estate has already ratified the general form of the remedy: **an exemption nothing checks is not
an exemption, it is a claim.** "No global priority between lanes" is currently a claim.

---

## What it actually tests, which is not the scheduler

The shuffle is easy to mis-describe as a scheduler test. It is not. It is **the falsifier for
`§13.3`'s point 2**:

```
POINT 2   ONE READ RULE AT BOTH SCOPES:
          YOU READ WHAT WAS PUBLISHED BEFORE YOUR LANE STARTED.
```

If that rule holds universally, **lane order cannot matter** — every lane reads a set of values
fixed before it ran, so permuting the lanes permutes nothing observable. If it is violated anywhere
— one lane reading a value another lane published *this* tick — then order decides the answer, and
the shuffle is what makes that visible.

**Point 2 is the assumption that makes point 1 safe, and point 1 is the thing the user is being
asked to rule on.** Testing the read rule is therefore worth more than testing the scheduler.

---

## What varies, and what is held

```
VARIES     the order in which DUE lanes are executed within one tick
HELD       the seed; every rate; WHICH lanes are due on a given tick; all params;
           the world; and -- critically -- the INTRA-LANE order
```

**Intra-lane order is not shuffled, and that is not a simplification.** `§13.1` ruled that the
authored order inside a lane **wins**, and that a consumer placed before its producer reads *last
tick's* value rather than being silently corrected. That makes intra-lane order **semantic**.
Permuting it would not be sampling a legal alternative; it would be authoring a different program.

The shuffle permutes exactly the set `§13.3` says has no priority, and nothing else.

---

## What it asserts, and why a digest is the right instrument

**The published-value set at end of tick must be identical across orders.** Not close — identical.

This is the same instrument this engine already uses twice, which is the reason to be confident it
is buildable rather than merely desirable:

| precedent | the claim it makes falsifiable |
|---|---|
| **two-world isolation digest** | two worlds in one simulation cannot influence each other. A single fold over registered state either reproduces or it does not, and a leak shows up as a moved number rather than as a plausible trajectory. |
| **CPU/GPU parity band** | two independent implementations of the same step agree **bit for bit**. Not within a tolerance — bits. A tolerance would have hidden every defect that band has actually caught. |

Both work for the same reason, and it is the reason to fold rather than to compare fields: **a
digest has no opinion about which differences matter.** A hand-written comparison of "the fields we
expect to be order-independent" can only confirm the expectations of whoever wrote it, and the
defect this is hunting is precisely an expectation nobody wrote down.

**Bit equality, not tolerance.** A shuffle that permits a small numeric difference is testing
nothing: any order-dependence large enough to matter starts out smaller than any threshold anyone
would pick, and grows.

---

## THE PREMISE GUARD, WHICH IS THE PART THAT WILL ACTUALLY BE SKIPPED

This class of test goes **silently inert** in at least three ways, and every one of them reports
green:

```
1  ONE LANE            a permutation of a single element is the identity. Shuffle reports
                       green having compared a run to itself.
2  NO CO-DUE LANES     if no tick ever has two lanes due at once, there is nothing to
                       permute, no matter how many lanes exist. Rates that share no common
                       multiple in the run's length produce exactly this.
3  THE SEED LANDS ON   a shuffle is allowed to return the identity permutation. On a short
   THE IDENTITY        run with few lanes this is not rare.
```

> ### A FILTER MATCHING NOTHING IS A FALSE GREEN; A SKIP IS VISIBLE.

So the mode must **assert its own premise before it asserts its conclusion**:

- **at least two lanes were co-due on at least one tick** — measured from the run, not assumed
  from the configuration;
- **the order actually executed differed from baseline** on at least one such tick;
- and the **seed is reported in the pass line**, not only on failure, so a green is attributable
  to a specific permutation rather than to "the shuffle ran."

A premise failure must be a **skip with a reason**, never a pass. The estate has paid for the
alternative: a guard whose subject list is empty, looping over nothing, printing green.

---

## What this cannot do, stated so nobody claims it later

**It cannot tell you whether the budget model is right.** That is the other half of `§13.3` and it
is a genuinely different problem:

> ### A CONSTANT COMPUTED FROM THE WRONG CLOCK IS STILL DETERMINISTIC, AND DETERMINISM IS WHAT THIS SUITE CHECKS.

A simulated per-lane execution time is an **input**. Every downstream number is self-consistent
with whatever was chosen, so a budget wrong by an order of magnitude yields a clean, reproducible,
bit-identical run that reports deadline overruns on a schedule bearing no relation to the target —
and `§12.3` puts that latency on screen. **The budget model's fidelity is unfalsifiable from inside
the budget model**, which is why `§13.3`'s real-concurrency mode is its **only oracle** and not a
hardware-in-the-loop nicety. The shuffle does not touch that and must not be offered as if it did.

**It samples; it does not prove.** With `n` co-due lanes there are `n!` orders and this will ever
see a handful. It is a falsifier, not a proof — which is the correct shape for the claim, because
the claim ("order never matters") is universal and therefore refutable by one case and confirmable
by none.

**It is blind outside the published-value set.** A component that writes a file, mutates a global,
or reaches past the publish/read surface is invisible to a fold over published state. That is a
real gap and it is the same gap the two-world isolation digest has.

---

## What would make this design wrong

- **If lanes are given shared mutable state.** The whole argument rests on `§12.5`'s reference
  model plus the frozen-snapshot read rule: lanes read *published* values and do not share writable
  memory. Break that and order-independence stops being a property worth asserting, because it
  stops being true — and the correct response is to fix the sharing, not to relax this.
- **If intra-lane order ever stops being semantic.** `§13.1` is what makes "shuffle lanes, never
  within a lane" the right line. If that ruling changes, this partition changes with it.
- **If a legitimate reason for cross-lane ordering appears.** Then "no global priority" is simply
  false, it should be said out loud, and this mode should be retired rather than weakened into a
  test that passes. **A guard relaxed until it passes is worse than no guard**, because it still
  reads as coverage.

---

## Implementation site

**`runtime/`, where lanes are scheduled — not `spade/`.** Spade contributes the instrument
(`state_digest`'s fold over the registered-array walk, folded in registration order) and the
discipline the two existing bands were built with. This note is the design and the argument; the
code, when `§13.3` is ruled, belongs to whoever owns the scheduler.
