# Spade -- Charter

**What this document is.** The founding document of the Spade spec set: what Spade is, where
its boundary runs, and the commitments every later Spade document inherits. It is a charter,
not an engine design -- it fixes identity, obligations and seams, and deliberately leaves
buffer layouts, batching, rate envelopes and module structure to the engine design spec.

**Harvested from** (read-only sources, superseded by this file):

- `superseded/kat-spade-simstack-foundation.html` -- "Spade & Sim Stack -- Foundation Charter", the
  seventh Kat spec, approved 2026-08-06. Carried in full except as noted under
  *Not carried forward*.
- `superseded/kat-spade-library-sandbox.html` -- sections 1 and 2 only (**`SL1`**, **`SL2`**,
  **`SL2a`**, **`SL2b`**). The rest of that spec (object model, behaviors, transfer audit,
  sandbox, quarantine, build/CI, phases) stays with the library/sandbox program.
- `superseded/kat-spade-host.html` -- section 2 only (**`HS2`**, the three engine-agnosticism rules).

**Ruling series owned here.** **`P1`**-**`P8`** (architecture pillars) and **`SA1`**-**`SA3`**
(amendments to the approved errata) live here and nowhere else. **`SL1`**, **`SL2`**,
**`SL2a`** and **`SL2b`** are resident here; the `SL` series identifier space belongs to the
library/sandbox program, which still cites them by name. **`HS2`** belongs to the sim-host
spec's `HS1`-`HS12` series; this document is the single canonical statement of the boundary
`HS2` names, and the host spec's rules are reproduced here verbatim in substance.

**Also owned here:** the three determinism grades (section 4). The sources state that rule
four separate times -- foundation `P2`, foundation `SA3`, the engine design spec's parity
section, and the execution record's `D11` row. It is stated once, here; the other copies are
restatements and should cite this section rather than re-deriving it.

**Not carried forward.** The foundation charter's milestone-placement section, its
2026-08-06 gap register, its "owed by the engine design session" list (discharged by the
ninth spec, 2026-08-08) and its survey/dead-code appendix are all execution-state records of
a survey now three programs out of date. They belong to the execution record, not to a
charter. Nothing normative was in them.

---

## 1. Identity

### 1.1 What Spade is

**`SL1`** -- **Spade is a domain-neutral blanket engine, normatively.** Its core --
`core`, `state`, `world`, `physics`, `render`, `sim`, `compute` -- carries **no domain
concept**. It knows bodies, elements, contacts, meshes, materials, lights, sensors and
worlds. It does not know drones, missions, gates, policies or packages.

Domain vocabulary lives in **convenience layers above the core** that construct general
descriptions and add no code path of their own. `vehicles/` and `sensors/` are exactly this
and stay where they are: `make_quadrotor()` constructs a `ModelType` from a dozen numbers,
and deleting it would cost callers convenience, not capability.

**The test of a proposed engine addition** is whether a caller with no aerial-vehicle
interest would still want it. A fluid solver passes. A gate-pass detector does not.

Why this needed saying normatively: the engine was *already* generic where it counts and
nothing said so. `vehicles/quadrotor.hpp`'s own header states that "there is no Quadrotor
class, no quadrotor-shaped code path in the `Simulation`, and no quadrotor branch in any
pass. If this file were deleted, the engine would still fly quadrotors." That discipline was
being held by comment and habit rather than by specification.

**Kat is one consumer of Spade, not its owner.** **`SL2a`** corrects the older framing
explicitly: the Spade README opened by defining Spade as "Kat's simulation engine", and that
definition is retired. The library/sandbox spec drops the `Kat --` title prefix that every
other document in the corpus carries for the same reason.

> **STALE:** the foundation charter's identity section said Spade "is *Kat's engine, cleanly
> bounded*: it exists to serve Kat's sim stack ... and spends no energy on standalone
> packaging or independent releases." Both halves are now false. `SL1` makes the engine
> domain-neutral by specification rather than by convenience, and `SL2`/`SL2a` give it
> install/export rules, an out-of-tree consumer built by CI on two platforms, a changelog and
> a standalone quickstart. The surviving half of that sentence is the *bounded* part, which
> section 5 states properly.

### 1.2 What Spade is technically

A **high-fidelity, low-latency, high-throughput physics and rendering engine**: complex
rigid-body collision, aerodynamics, extreme-accuracy step rates -- written primarily in GPU
compute, operable CPU-only, optimized for GPU.

**v2 is a modification/addition/refine stage, not a rewrite.** The 2026-08-06 survey
confirmed the foundations fit: the data-oriented ECS, the compute-pass structure, the
`std430` POD state layout (memcpy-able by design), and the atomics-free
one-invocation-per-index shader discipline -- the exact property that makes same-device GPU
determinism achievable -- all carry forward. What changed was plumbing and discipline, not
architecture.

### 1.3 The sim stack, and where the seam is

```
editor viewport - kat-train rollouts - kat CLI
        |  C5 -- the sim-host contract (errata R1: surface, conformance, semver)
        v
dronesim            // implements C5. Thin adapter: contract surface, drone
        |           // assembly (drone description -> engine bodies/aero/actuators),
        |           // Kat-side timing and sample stamping. No physics of its own.
        v
spade/              // the engine. Owns rigid bodies, collision, aerodynamics,
                    // propulsion/actuator force models, sensor-simulation
                    // primitives, rendering, worlds. No Kat types in its API.
```

**Boundary ruling: fat Spade, thin dronesim.** Everything that can live in the engine does --
including aerodynamics, propulsion force models, and sensor-simulation primitives (IMU
synthesis, camera render-to-sensor). dronesim contributes no physics: it is the C5 surface,
the mapping from a drone description onto engine constructs, and Kat-side stamping. This
maximizes what Spade grows into while keeping the contract seam exactly where the errata put
it.

Consequence worth stating: because dronesim is thin, C5 conformance effectively tests Spade
through a narrow adapter -- engine regressions surface as contract failures, which is the
intended coupling.

The Spade-backed adapter itself is Kat-side by definition and lives at `dronesim/spade/`
(sim-host spec `HS1`); hosting it inside `spade/` was rejected on this charter's grounds --
the engine tree stays kat-free.

---

## 2. Architecture pillars

Eight commitments, decided 2026-08-06. Each is a rule plus the argument that produced it.

### `P1` -- CPU reference twin

Every authoritative physics pass exists twice: GPU compute (primary, performance) and CPU
(reference), kept in **enforced tolerance-banded parity** by the fixtures pattern this
project already lives by.

The twin is **not a degraded fallback**: it is a full implementation and it is the
determinism reference. That is what makes section 4's grade structure possible at all -- a
portable, bit-identical reference path is the only thing a golden corpus can be recorded
from.

### `P2` -- Three-grade determinism

CPU bit-identical per platform (the conformance grade), GPU bit-identical per device+driver,
CPU<->GPU tolerance-banded. Stated in full in section 4; amends C5 via `SA3`.

### `P3` -- Vulkan

The GPU path moves to Vulkan: true headless compute, offscreen render, explicit sync, a
Linux/cloud-training route, and GLSL->SPIR-V carries the existing shader logic forward rather
than rewriting it.

> **STALE:** `P3` closed with "GLFW/GL survive only as long as the port takes." They outlived
> it in a different role -- `SL14a` moves GLFW and ImGui from the `SPADE_BUILD_V1` gate to
> `SPADE_BUILD_SANDBOX` (GLAD leaves the build entirely), so GLFW survives as the sandbox's
> windowing dependency, not as an engine path. The ruling stands; the sunset clause does not.

### `P4` -- Fixed-step, seeded

Fixed-step only; **no wall-clock read anywhere in the step path**; every RNG seeded and
domain-separated in the R4 style; `dt` immutable after creation, per C5. This is the
precondition for everything in section 4 -- a step path that reads a clock cannot be replayed,
and an unseeded RNG cannot be a reference.

### `P5` -- Thin adapter above

The section 1.3 boundary: Spade owns the physics and sensor primitives; dronesim owns the
contract surface and assembly. **No Kat types below the C5 seam.** See section 5 for how this
is enforced.

### `P6` -- Offscreen render boundary

Spade renders the world to an offscreen image; the editor imports it and composites UI on
top. Headless is simply not rendering. **Stepping never depends on render state** -- a C5
conformance test, not an aspiration, because a step path that reads render state cannot run
headless and cannot be replayed.

### `P7` -- Effects layer, with a roadmap to the aero path

The SPH/grid/particle machinery survives as the **non-authoritative** effects/medium layer,
with a stated roadmap toward air-flow simulation (downwash, prop wash, wind fields).

**Promotion rule:** nothing becomes authoritative until the CPU twin can mirror it -- `P1`
applies to every authoritative pass, no exceptions. That rule is what keeps the effects layer
from quietly becoming physics no reference path can check.

*Reconciliation:* `P7` was written when that machinery was v1's. `SL14a` quarantines v1 to
`spade/legacy/` and cuts it from the build entirely, and `SL14b` requires that **every system
v1 holds must exist in v2 before the quarantine happens** -- so the capability survives by
re-implementation, not by the v1 code surviving. `SL14c` additionally retains the
50,000-body `fluid` scene as a standing capability target.

### `P8` -- Snapshot as first-class

World snapshot/save/restore is a core engine feature (C5 world-snapshot, errata R9), sized
and designed for the editor's session ring and for training determinism -- **not an
afterthought serializer**. Retrofitting serialization onto a GPU-resident world is a design
problem (readback), not a coding chore, which is why it is a pillar.

---

## 3. Amendments to the approved errata -- `SA1`-`SA3`

Three decisions made in the foundation session supersede specific errata statements, per the
docs-spec supersession rule.

**Namespace note** (batch 2 section 0, 2026-08-08): these were originally numbered A1-A3 and
are retroactively **`SA1`**-**`SA3`** -- the editor technical spec's amendment series is
TA1-TA6, and bare "A*n*" citations are retired. Cite these as `SA1`-`SA3`.

### `SA1` -- Spade is the M1b backend

*Errata R11/M1b said:* "C5 + dronesim: **interim C++ physics** behind the sim host."

*Now reads:* **Spade is the M1b backend.** No interim C++ physics backend is built. The
current KAT sim code is still mined for reference and test oracles, but never becomes a C5
implementation.

**The accepted risk, stated rather than hidden:** M1's critical path now runs through engine
work. What buys that back -- M1a (runtime skeleton) has no world dependency and proceeds in
parallel; the CPU twin lands first and is dramatically less work than the Vulkan port; and
building one backend instead of two deletes an entire throwaway implementation.

### `SA2` -- the parity pair is Spade's own two paths

*Errata R1 said:* "The two conformant backends at M1b are the interim C++ physics and
dronesim-on-Spade."

*Now reads:* the cross-backend parity pair is **Spade's CPU twin <-> Spade's GPU path** --
two implementations of the same passes inside one engine, compared through the same C5 seam.
`golden-parity`'s widened definition ("any two implementations of a Kat contract") already
covers this; the twin is the second implementation. `SA2` is the direct consequence of `SA1`:
once there is one backend, the second implementation must come from inside it, and `P1`
already put one there.

### `SA3` -- three determinism grades

*Errata R1 said:* C5 conformance is "bit-identical body state and sensor output, per
platform."

*Now reads:* the three grades of section 4. `SA3` is the single point where C5's conformance
claim was narrowed to what is actually obtainable.

---

## 4. Determinism -- the three grades

This is the canonical statement. `P2`, `SA3`, the engine design spec's parity section and the
execution record's `D11` row all say this; they are restatements of what follows.

| Grade | Claim | What it buys |
| --- | --- | --- |
| **CPU path** | Bit-identical **per platform** | The C5 conformance claim and CI's gate. **Golden traces are recorded from the CPU path only.** |
| **GPU path** | Bit-identical **per device + driver** | Same-machine replay and editor step-back. Not a cross-machine claim and never presented as one. |
| **CPU <-> GPU** | **Tolerance-banded** parity | The two paths are held to each other continuously, without pretending a GPU reproduces a CPU bit for bit. |

**Why three and not one.** Bit-identity across GPU devices and drivers is not obtainable --
scheduling, driver code generation and floating-point contraction differ between them.
Demanding it in the conformance claim would force one of two bad outcomes: a claim that is
quietly false, or the GPU path pushed out of the product. Anchoring conformance to the CPU
twin instead makes the claim provable on any machine and makes the golden corpus portable.
This is the payoff of `P1` -- the twin being a full implementation rather than a fallback is
exactly what makes it usable as the reference.

**Why the GPU grade is achievable at all.** The atomics-free, one-invocation-per-index shader
discipline, and a bitonic sort that is a fixed network rather than a data-dependent one, make
same-device runs order-deterministic *by construction* -- not by luck and not by a sorting
pass bolted on afterwards.

**How the bands are set.** Per-quantity tolerance classes (positions, velocities, quaternions,
RPM), compared abs-OR-rel with relative semantics applying where the CPU value is non-zero.
Bands are **measured, then pinned with margin**, and cross-platform calibration happens
*before* pinning -- the consolidation's golden-atol lesson, standing policy. Device provenance
is recorded alongside the pinned bands, because a band without the device it was measured on
is not evidence.

**What this depends on.** `P4` entirely: fixed step, no wall-clock read in the step path,
seeded and domain-separated RNG, immutable `dt`. And `P6`: stepping never depends on render
state.

### 4.1 The gaussian path -- a declared band on an *input*, not an output (`R3`, 2026-09-24)

The third row above already covers this divergence. What was missing until `R3` is that it was
**undeclared** -- licensed by a deferral in `fp32_math.slang` carrying **no owner and no
trigger** since it was written. That deferral is now deleted and the grade is stated here.

> **The gaussian draw is CPU<->GPU tolerance-banded.** `rng`'s Box-Muller `sqrt` is the sole
> bander of the gaussian's float half -- `rng.slang` states this as design, not as a
> deduction. `log32`/`sin32`/`cos32` are proven bit-identical to the host and the integer
> state is exact. **Only the `sqrt` differs, and only because Vulkan specifies it to <= 2.5
> ulp where IEEE 754 mandates correctly rounded.**

| | |
| --- | --- |
| **measured** | max abs `4.76837158e-07`, max rel `5.06893741e-07` |
| **n** | 64 ring elements, **11 outside** a zero band |
| **device** | Intel Iris Plus Graphics, Vulkan 1.3.215, driver 31.0.101.2125, msvc-ninja-release |
| **isolation** | GNSS receiver, 200 substeps, `bias_tau_s = 1e-6` (retention underflows to exactly zero) and `mount_pos = 0` (orientation cannot reach the report), so every surviving term is a draw times a byte-identical constant |
| **scope** | **by call, not by include**: `dryden.slang` - `sensor_gnss.slang` - `sensor_imu.slang`. `forces_drag` and `medium_update` include `rng` and never call the gaussian. |

**11 of 64 rather than 64 of 64, at roughly 1 ulp.** That is what a <= 2.5-ulp licence looks
like rather than a systematically different algorithm, and it is what makes banding this
honest rather than resigned.

**Tightening is refused, and not on cost.** Vulkan does not guarantee a correctly-rounded
`sqrt`, so a "tightened" implementation would be **a claim the platform cannot keep**.
Respelling it as a polynomial would trade a <= 2.5-ulp difference for a *guaranteed* one.

#### Why this one does not fit the band model above, and what that costs

The paragraph on how bands are set describes **per-quantity tolerance classes** -- positions,
velocities, quaternions, RPM. Every one of those is an **output**. The gaussian is an
**input**, and the two compose differently:

> **A BANDED INPUT REACHING THREE KERNELS PRODUCES BANDS IN MANY OUTPUT QUANTITIES, AND
> BANDING EACH OUTPUT SEPARATELY HIDES THE COMMON CAUSE.** Each per-quantity band then reads
> as a property of its own kernel, and widening any one of them looks local when it is not.

So this grade is recorded **at the source** -- `rng.slang`'s header carries the measurement --
and downstream bands cite it rather than re-deriving a cause. That is exactly the defect
`kGnssPosition` carried twice before it was measured: a real number beside a guessed cause,
where the cause belonged to something upstream.

**A declared band and a live reproduction are different instruments, and this path keeps
both.** `tests/test_gpu_parity.cpp`'s `GnssDrawsDivergeAcrossBackends_KNOWN_OPEN` asserts the
divergence **is** present, so it goes red the day anyone tightens the `sqrt` and demands its
own retirement. A declaration alone would outlive the thing it describes.

#### Two things this grade does *not* settle

**It does not touch the C5 conformance claim.** That claim is anchored to the **CPU path**
(section 4's first row -- golden traces are recorded from the CPU path only), and
`runtime/04-host-c5.md`'s `R1` states bit-identity **per platform**. Nothing customer-facing
depends on bit-exact sensor draws *across backends*, which is why `R3` did not pause.
**If such a claim is ever written, it reopens this grade** -- that is a `C5` contract question
and not Spade's to decide alone.

**It acquires a declared consumer under `R2`.** Once `wire::GnssFix` carries
`sigma_h`/`sigma_v`, a component will weight a fix by an accuracy number that is itself
downstream of a banded draw. `R2` does not create that composition and this grade does not
forbid it -- but **a banded input feeding a weighting term is a different risk shape from a
banded output being read**, and it is named here before anything fuses on it.

---

## 5. The kat-free boundary

The boundary runs in **both directions**, and each direction has a different failure mode.

**Rule 1 -- Spade stays kat-free** (`HS2` rule 1). The engine tree gains zero Kat
dependencies, schemas or concepts. Everything Kat-flavoured lives in the adapter.

**Rule 2 -- the Kat side stays Spade-free** (`HS2` rule 2). No Spade header is includable
outside `dronesim/spade/`. Every consumer -- editor, trainer, SDK -- reaches simulation only
through C2/C5 and schema'd artifacts.

**Rule 3 -- connectors bind to schemas, not structs** (`HS2` rule 3). The adapter is a
*resolver*: it consumes the schema'd configuration artifacts and maps them to Spade PODs
privately. **A different engine would implement the same resolution against the same
artifacts** -- that is the whole point of the rule, and the reason it is stated at the
connector level rather than left implicit in rule 2.

**Rule 4 -- no Kat-shaped concept, even in Kat-free form** (`SL1`). This is the direction that
had never been stated. The engine may not grow a Kat concept merely by renaming it into
engine vocabulary. A gate-pass detector with no `kat_` symbol in it still violates the
charter. **A guard cannot catch this; only a specification can** -- which is why `SL1` exists
as text and has no test.

### Where it is enforced

1. **Build-level separation.** Spade keeps its **own CMake project**, built as a subproject.
   Kat code consumes Spade's public API only; no Kat target includes `spade/src` internals.
2. **CMake-level header ban.** No Spade header includable outside `dronesim/spade/`, enforced
   in the same mechanism class as the editor's `kat_assert_no_widgets()`, and mirroring the
   existing "no `kat_*` symbol outside `core/sim/` + `core/validate/`" rule.
3. **`tools/tests/test_engine_agnosticism.py`**, which enforces rules 1 and 2 as a test. It
   is **live, not decorative**: it went red during the render program's Task C0 and caught a
   real violation.

Rule 4 has no enforcement point by construction (see above). Section 6's `SL2b` guard is a
fourth mechanism serving a related but distinct purpose -- proving the *public surface* is
sufficient, not proving the boundary is clean.

### The format-layer caveat -- G2 stands

Engine-agnosticism holds **at the contract layer, not the format layer**. The world file and
the sim-physics vocabulary remain **Spade-defined, versioned as C5 contract artifacts**
(config spec G2; world schema v1 user-frozen 2026-08-12). Any conforming host must read them.
**No engine-neutral world format is created** -- inventing one would buy portability nobody
asked for at the cost of a translation layer nobody could keep honest.

---

## 6. The library surface and its hardening

Spade is **already a well-formed installable library**, and `SL2` is deliberately narrow
because of it: eleven modules, each a static library with a `spade::` namespaced alias,
`install(TARGETS ... EXPORT spadeTargets)`, per-module header installation, and a generated
`spadeConfig.cmake`.

An out-of-tree `find_package(spade CONFIG REQUIRED)` consumer project is configured, built and
run by CI on **both Linux and Windows** against a scratch install prefix. It is **load-bearing
rather than ceremonial**: it is what caught the `yaml-cpp` consumer-link gap that the in-tree
tests structurally could not see, because a static archive only extracts the members it needs.

**Severability is proven, not aspirational.** `scripts/spade-prefix.ps1` has built Kat's Spade
install with `-DSPADE_BUILD_V1=OFF` from the beginning: **Kat has never linked v1.**

### `SL2a` -- release identity

Spade gains its own release identity: `spade/CHANGELOG.md`, a stated versioning policy for
`project(Spade VERSION ...)`, and a `spade/README.md` quickstart that **stands on its own** --
buildable, installable and consumable by a reader who has never heard of Kat. The README's
"Kat's simulation engine" opening is corrected (section 1.1).

**Not in scope:** a package registry, binary distribution, or an ABI-stability promise. Those
belong to seat S1 if they are ever wanted.

### `SL2b` -- the sandbox may use only Spade's public installed surface

`spade_sandbox` may include only headers that `install(DIRECTORY ...)` actually installs, and
may link only `spade::` targets. It may not reach into engine internals by relative path, and
it may not depend on anything the exported package does not carry.

**Why this is the highest-value constraint in that spec:** the sandbox is the library's
largest consumer, so the rule turns it into a **continuous proof that the public API is
sufficient**. If the sandbox needs something the API does not expose, that is a *finding about
the library*, not a reason to widen an include path. Enforced by a guard over the sandbox's
include graph, in the same shape as the agnosticism guard.

---

## 7. Repo relationship

### Executed 2026-08-06

- **Subtree merge:** `desktop/Spade` -> `spade/` at commit `1c8a133`, all 14 commits of
  history preserved (second parent `2d36c54`).
- **README corrected** (`4cca21d`): the survey's drift list fixed -- C++20 / CMake 3.15 /
  GL 4.3 core / FetchContent-not-vendored / `Sandbox` binary / real API signatures.
- **Git control:** Spade's own `.gitignore` was already adequate and is kept; Kat's root
  ignores were verified non-conflicting (`cmake-build-*/` and `.idea/` unanchored; the root
  `/shaders/` rule cannot touch `spade/assets/shaders/`).
- **`design-specs/` untracked** (`6f3abb0`): specs stay hand-authored, local and
  artifact-backed, gitignored.

### The copy under `spade/` today

185 tracked files, subtree-merged, **no submodule, not ignored**, and it is **the development
home**: Spade is built and tested inside Kat for development convenience and is freely
editable there. "Harden in place" means hardening the *development* home; it is not a claim
that no external Spade exists.

> **RESOLVED (`C6`):** `SL2a` governs -- the external repository is **live and unrelated to Kat**.
> It is the later statement and a direct user ruling; the charter's "frozen archive awaiting
> retirement" is the older view, superseded, and kept in `superseded/` as provenance. This matters
> because `SL14a`'s quarantine-not-delete argument rests on v1's history surviving in two places --
> under the operative reading it does, so that argument stands on the ground it claims.

### Standing rules

- `spade/` joins the docs-spec section 3 **infrastructure-directory exemption** (enumerated
  list; proper names are not domain names).
- Spade keeps its **own CMake project**, built as a subproject. Kat code consumes Spade's
  public API only; no Kat target includes `spade/src` internals. (Enforcement point 1 in
  section 5.)
- Spade versioning: the foundation pinned `0.1.0` until v2 versioning was established;
  addendum A11 (signed 2026-08-10) advanced it to `0.2.0` at the S4-M1B claim. `SL2a` records
  `0.2.0` as still "a bare version with no release notes behind it" and makes the versioning
  policy and changelog its own deliverable.
- Spade's own history belongs to Spade. `SL14a` places the v1 quarantine at `spade/legacy/`
  rather than the repo-root `legacy/`, on the grounds that root `legacy/` is a Kat concept and
  **`SL1`'s identity claim would be hollow if the engine's predecessor were archived as a Kat
  artifact.**
