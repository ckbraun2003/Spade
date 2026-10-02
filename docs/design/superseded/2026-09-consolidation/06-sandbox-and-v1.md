# Spade -- The Sandbox, the v1 Transfer Audit, and v1's Retirement

**What this document is.** The single canonical statement of how spade v1 leaves the tree
and what replaces it: the v1-to-v2 transfer audit that must reach zero open rows first, the
`spade_sandbox` reference application that becomes the way to look at the engine, the
quarantine of v1 to `spade/legacy/`, the scene-equivalence obligation that has to be
discharged while v1 still builds, and the build/CI shape that lets a windowed tool be
tested. Passes, buffers and parity bands belong to the engine design spec; the object model
and behaviors stay with the library program.

**Harvested from** (read-only sources, superseded by this file):

- `superseded/kat-spade-library-sandbox.html` -- the twenty-fourth spec, drafted 2026-08-30, DRAFT.
  Sections 5-8 only: **`SL7`**, **`SL8`**, **`SL9`** and **`SL9a`**-**`SL9f`**;
  **`SL10`**-**`SL13`**; **`SL14`** with **`SL14a`**/**`SL14b`**/**`SL14c`**; **`SL15`**
  with **`SL15a`**/**`SL15b`**. Sections 1-2 (`SL1`, `SL2`, `SL2a`, `SL2b`) live in
  `01-charter.md`; sections 3-4 (`SL3`-`SL6`) stay with the library program.
- `superseded/kat-spade-render-scene.html` -- section 10 only: **`RS10a`**, **`RS10b`**, **`RS10c`**.
  The twenty-first spec, drafted 2026-08-24, still a live DRAFT.
- `spade/docs/v1-transfer-register.md` -- the as-built normative register, machine-checked
  by `spade/tests/test_transfer_register.cpp`. **This file is ground truth for the transfer
  audit**; where the twenty-fourth spec's prose and the register disagree on a disposition,
  the register wins, because Plan A landed after the spec was drafted.

**Ruling series owned here.** **`SL7`**-**`SL15b`** are resident in this document and
nowhere else; the `SL` identifier space belongs to the library/sandbox program, which still
cites them by name. **`RS10a`**, **`RS10b`** and **`RS10c`** belong to the render program's
`RS1`-`RS15` series and are adjudicated here because the twenty-fourth spec's banner
explicitly absorbs them.

✅ **Status: the rulings are SIGNED 2026-09-17** (`SL1`-`SL18`, ~30 ids; `RS10a`/`RS10c` within the enumerated `RS` 20, `RS10b` excluded). ⚠ **Both PARENT DOCUMENTS remain DRAFTs and always will** -- they are superseded sources, and their status is a fact about those files, not about the rules. Plan A has landed on master;
Plan B (SPH fluid) is unwritten; Plan C (the sandbox) is **written 2026-09-17** (`plan-c-sandbox.md`) and **unexecuted**.

<!-- signoff-state: SL1-SL18 SIGNED -->
<!-- ^ Read by tools/tests/test_spade_sandbox_gate.py (the PC21 gate), and by nothing else.
     While this says UNSIGNED, no CMake target named `spade_sandbox` may exist in the build
     graph -- the user ruled SIGN SL1-SL18 BEFORE PHASE C CODE STARTS (07-status.md).
     On sign-off, flip this one word to SIGNED: the guard then changes polarity rather than
     being deleted, and docs/dev/executables.md takes over asserting the target EXISTS.
     The guard reads THIS TOKEN and never the word DRAFT, which appears six times in this
     file -- five of them permanent history about superseded parent specs. -->


---

## 0. Two things a reader must know before reading anything else

This is the estate's sharpest contradiction and its most consequential open gate. Both are
stated here rather than in the sections they belong to, because a reader who reaches either
parent spec first gets the wrong instruction and nothing inside those specs points forward.

> **SUPERSEDED:** **`RS10b`**'s "deleted" is dead text. **`SL14a`** replaces it with
> quarantine.
>
> `RS10b` (twenty-first spec, section 10) says: "D10's strangler ends here: `spade/src/`,
> `spade/include/`, `spade/examples/`, `spade/assets/`, the `SPADE_BUILD_V1` option,
> `tools/viewer/bridge.cpp` and the vendored GL dependency chain **are deleted**." It
> repeats the instruction twice more inside its own spec -- in `RS11`'s executable-register
> rows (`spade_viewer` RETIRED, `Sandbox` "RETIRED with v1") and in `RS12` checkpoint 4,
> whose gate reads "v1 tree **deleted**; build green with no GL dependency."
>
> `SL14a` (twenty-fourth spec, section 7) rules on the same paths: they "move to
> `spade/legacy/` as **pure renames with zero edits**, following the repo's existing
> quarantine precedent exactly." This is user ruling 7 of the 2026-08-30 session, recorded
> in that spec's section 0: "v1 is quarantined to `spade/legacy/`, **not deleted**, and
> completely cut off from any functionality or system."
>
> **Quarantine is operative.** SL is the later spec (2026-08-30 vs 2026-08-24) and its
> banner explicitly claims the amendment: it "absorbs RS10a/RS10b/RS10c (21st spec Phase 4)"
> and "retires spade v1 to `spade/legacy/` (D10 endpoint)". Nothing in the render spec
> points forward -- `RS14`'s amendments-owed table does not mention the twenty-fourth spec
> at all -- so the render spec reads as authoritative to anyone who opens it alone. It is
> not, on this clause.
>
> `RS11`'s register rows survive under a re-reading: "retired" is true of `spade_viewer` and
> `Sandbox` in the sense that both are cut from the build and no longer runnable. `RS12`
> checkpoint 4's word "deleted" does not survive; it should read "quarantined and
> unreachable", which is what `SL17`'s Phase P7 checkpoint already says.
>
> **Path disagreement, resolved against the tree.** `RS10b` names
> `tools/viewer/bridge.cpp`; `SL14a` names `engine/tools/viewer/`. **SL is correct.** There
> is no `spade/tools/`; the viewer is at `spade/engine/tools/viewer/`, holding `bridge.cpp`,
> `bridge.hpp`, `main.cpp` and `scenes.cpp`. `RS10b` also names only `bridge.cpp` where
> `SL14a` moves the whole directory -- and `SL14a` is right to distinguish them, because
> `scenes.cpp` is v2 code that must be harvested, not quarantined (section 4.1).

> **BLOCKED:** the quarantine has **not** been executed and **must not** be, because the
> register still has an open row.
>
> `spade/docs/v1-transfer-register.md` carries one `to-transfer` row: **SPH fluid**
> (`EnableSPHFluid`, `FluidDensity.comp`, `FluidForce.comp`, `FluidComponent`,
> `FluidMaterial`). Its evidence cell reads, verbatim: "SL8 -- **THE ONE OPEN ROW.** v2 has
> no fluid solver of any kind, and this is what the 50,000-body fluid scene runs on."
>
> Every other row is closed: nine `transferred`, three `retired-with-reason`, one
> `retired-to-sandbox`. The register's own closing section states the consequence: "One row
> is open: **SPH fluid**. Until Plan B closes it, section 7's quarantine of v1 must not
> execute -- v1 is still the only implementation of the fluid solver, and quarantining it
> would retire a live capability rather than a superseded one."
>
> On-disk state confirms nothing has moved: `spade/src/`, `spade/include/`,
> `spade/examples/` and `spade/assets/` are all present, there is no `spade/legacy/`, and
> `spade/CMakeLists.txt:38` still declares `option(SPADE_BUILD_V1 "Build the v1 GL engine
> (src/, include/, examples/, assets/)" ON)`. The gate is holding.

---

## 1. `SL7` -- the transfer register

### 1.1 The rule

**`SL7`** -- **v1 may not be quarantined while it is the only implementation of anything.**
Every v1 system is dispositioned. **A silent drop is not a disposition.**

The D10 strangler had been running since S1 and was never audited for completeness. The
twenty-fourth spec's section 5 is that audit, and it found a real gap -- which is the whole
argument for doing it: an unaudited strangler looks exactly like a finished one right up to
the moment something is lost. The register "is verified against the v1 tree, not recalled",
and the as-built file repeats that provenance verbatim.

### 1.2 The four-value disposition vocabulary

| Disposition | Meaning |
|---|---|
| `transferred` | A v2 implementation exists and is exercised by tests. |
| `to-transfer` | Still open. v1 remains the only implementation; quarantine is blocked. |
| `retired-with-reason` | Deliberately not transferred, with the reason recorded here. |
| `retired-to-sandbox` | Leaves the engine by design; the capability survives in the sandbox. |

Four values, not two, because "not transferred" is three different situations that a
two-value vocabulary would flatten into one: `retired-with-reason` is a decision,
`to-transfer` is a debt, `retired-to-sandbox` is a relocation. Only `to-transfer` blocks the
gate. The vocabulary exists so the gate can be evaluated mechanically instead of read.

### 1.3 The register is machine-checked, and that is the point

The register is parsed by `spade/tests/test_transfer_register.cpp`, whose header states the
reason in one line: **"A register nobody parses is a document; a register a test parses is a
gate."** A document records an intention; a gate refuses an action. The difference matters
precisely at the moment the quarantine is about to run, which is the moment nobody re-reads
the document -- and the moment at which reading it wrongly costs a working capability
permanently, because once v1 is cut from the build it can never be run again.

Four checks, each shaped deliberately:

- `EveryRowIsDispositioned` -- every disposition is one of the four values, with the parse
  asserted **non-empty first**, so a parser that reads nothing cannot pass vacuously.
- `HasEveryRowTheSpecEnumerates` -- the row count is pinned at 13 and every row must carry a
  non-empty surface and evidence. Pinning the count is what stops a truncated file, a
  renamed heading or a parser that stops early from turning the suite green by reading fewer
  rows than the register holds.
- `KnowsSphIsStillOpen` -- a **positive** assertion that the SPH row reads `to-transfer`.
  Its comment states the failure mode: "If this ever reads `transferred` without Plan B
  having landed, the register is lying and Plan C would quarantine v1 over a live gap."
- `RecordsTheRowsPlanAClosed` -- positive assertions on the four rows Plan A settled, "so
  that a later edit which quietly reopens one is a failure here rather than a surprise at
  the quarantine gate."

Two implementation notes survive because both are defects the estate keeps re-finding. The
parser starts only after the `## Register` heading, because the file also holds a vocabulary
table of the same shape that a whole-file sweep would silently ingest. And a row splits into
five tokens on LF and six on CRLF; requiring six rejected every row and reported an **empty**
register -- caught only because `EveryRowIsDispositioned` asserts non-emptiness first.

`SL18` sets the standard the guard has to meet: it exists "so a future v1 system cannot be
quarantined unnoticed", and every guard the program adds "must be shown to fail under the
mutation it exists to catch".

---

## 2. `SL8` and `SL9` -- the per-system dispositions

### 2.1 The register as built

Ground truth, from `spade/docs/v1-transfer-register.md`. Thirteen rows.

| v1 system | v1 surface | Disposition | Reason / evidence |
|---|---|---|---|
| Gravity | `EnableGravity` | `transferred` | Applied inside Integrate for exact specific-force capture; the Gravity pass is deliberately inert (engine design A9). Pinned by `Schedule.GravityIsAppliedExactlyOnce`. |
| Motion / integration | `EnableMotion`, `Motion.comp` | `transferred` | `physics/integrator.hpp` -- symplectic Euler plus quaternion exp-map. |
| Grid collision | `EnableGridCollision`, `Grid*.comp` | `transferred` | `physics/grid` plus the CollisionDynamic pass; v1's hash-collision bug fixed by exact cell compare. |
| Bitonic sort | `BitonicSort.comp` | `transferred` | The sorted-grid chain; the Vulkan recorder records its dispatch chain once per shape and reuses it. |
| Wireframe / colour render | `RenderWireframe`, `RenderColor` | `transferred` | `DrawMode::wireframe` / `DrawMode::shaded` in `render/target.hpp`. |
| Newtonian gravity | `EnableBruteForceNewtonianGravity` | `retired-with-reason` | Already retired before the audit: the shader existed but the C++ body was never implemented. Deleted during S1 as documented dead weight. |
| Brute-force collision | `EnableBruteForceCollision` | `retired-with-reason` | **`SL9a`**. |
| Custom shader loading | `RenderShader(frag, geom)` | `retired-with-reason` | **`SL9b`**. |
| SPH fluid | `EnableSPHFluid`, `FluidDensity.comp`, `FluidForce.comp`, `FluidComponent`, `FluidMaterial` | `to-transfer` | **`SL8`** -- the one open row. |
| Velocity render mode | `RenderVelocity`, `Velocity.frag` | `transferred` | **`SL9c`**, Plan A Task 10. |
| Camera component | `CameraComponent` | `transferred` | **`SL9d`**, Plan A Task 2. |
| Input component | `InputComponent` | `retired-to-sandbox` | **`SL9d`**. |
| Instancing helpers | `SpawnInstancesInSphere/Cube`, `SetVelocity/Color/Mass`, `RandomizeVelocity/Color` | `transferred` | **`SL9e`**, Plan A Task 11. |

The twenty-fourth spec's prose still marks `SL9c`, `SL9d`'s camera half and `SL9e` as "to
transfer". That is staleness, not contradiction: Plan A landed after the spec was drafted
and closed all three. The register is the as-built record and is what the guard reads.

### 2.2 `SL8` -- SPH fluid is implemented to v2 discipline, not ported

**`SL8`** -- the one open row, and the largest single item in the program. It gets its own
phase (P4) and its own user checkpoint.

v1's fluid is a pair of GPU-only compute shaders with no CPU path, no determinism story and
no parity obligation. **v2 admits no such pass.** So the transfer is not a port; it is an
implementation under the engine's existing rules: a declared pass with both `execute_cpu`
and `record_gpu`, identical math and operation order, fp32 on both, seeded from the
domain-separated streams, and participating in the golden-corpus parity comparison with its
own ratified tolerance band. Scope: neighbour search reusing the existing sorted grid rather
than a second spatial structure; density and pressure-force passes; a `Fluid` component
carrying the material parameters; and the parity band **measured, never assumed**.
`ComponentTypeId::fluid` is already reserved so Plan B does not renumber the enum.

**Cost if wrong, stated in advance.** If SPH cannot reach an acceptable CPU-to-GPU band, the
honest outcome is a CPU-only fluid declared under `SL6`'s rule -- visible in the sandbox,
excluded from GPU-authoritative worlds -- **not a widened band**. Widening a band to
accommodate a new pass would compromise the corpus for every pass already in it. The
decision is written down before the measurement exists because the moment to choose between
a narrower capability and a weaker corpus is not the moment the number comes back.

### 2.3 `SL9a`-`SL9f` -- the retirements and their reasons

**`SL9a`** -- **brute-force collision: retire with reason.** Superseded by the sorted-grid
path, which is strictly better at every body count v1 shipped. Recorded so it is a decision
rather than an omission: nothing about the code changes, but a future reader finds a reason
instead of a hole.

**`SL9b`** -- **custom shader loading: not applicable, deferred to S7b.** A user-supplied
fragment or geometry shader has no meaning against a CPU rasterizer. Reserved for the Vulkan
backend. This does not block quarantine, because there is nothing v1 can do today that v2
cannot also do for any consumer that exists.

**`SL9c`** -- **velocity render mode: transferred.** `DrawMode::velocity` plus
`RenderOptions::velocity_scale_mps`, with `BodyPose::velocity` feeding `DrawItem::speed_mps`.
Pinned by `test_render_velocity.cpp`.

**`SL9d`** -- **camera and input split at the engine boundary.** `CameraComponent` transfers
as an engine component and becomes one of `SL5`'s ten registered types
(`ComponentTypeId::camera`, with `fov_degrees` / `near_plane` / `far_plane` / `active`); v2
already had `render::Camera` as a value but no camera-as-component. `InputComponent` does
**not**. It is the register's only `retired-to-sandbox` row: key bindings and fly-through
speed are an application concern, "the engine is headless by construction (`SL1`) and must
not grow an input concept." The capability survives as the sandbox's camera controller (Plan
C). This is the disposition vocabulary earning its keep -- the capability is neither
transferred nor dropped, and a two-value vocabulary would have had to lie about it either
way.

**`SL9e`** -- **instancing helpers: transferred.** `spawn_in_sphere` / `spawn_in_cube` with
`mass` and `velocity_radius_mps`, seeded from domain-separated splitmix64 streams rather
than v1's `std::random_device`-seeded mt19937 -- a determinism upgrade taken during the
transfer, not a like-for-like port. Directly what the sandbox's "spawn 50,000 bodies in a
sphere" affordance needs. One sub-item is explicitly **not** transferred and says why:
**colour has no body-side equivalent to transfer to.** In v2 colour is a `RenderScene`
`Material`, not a property of a body, so `SetColor`/`RandomizeColor` have no destination --
and recording that is the difference between a considered non-transfer and a silent drop.

**`SL9f`** -- **barycentric wireframe (`Barycentric.geom`): retire with reason.** A
geometry-shader technique for GPU wireframe overlay; the CPU rasterizer draws wireframe
directly and needs no equivalent.

> **FIXED 2026-09-07.** `SL9f` (barycentric wireframe) was enumerated by the 24th spec's `SL7` table
> but had never been transcribed into `spade/docs/v1-transfer-register.md`, which carried 13 rows
> against the spec's 14. `test_transfer_register.cpp` pinned the count at `13u` -- a number read off
> the register itself, so the guard agreed with the omission instead of catching it.
>
> **What was done:** the row was added (`retired-with-reason`), the pin raised to `14u`, and the
> register's header now states that **it is the authority, not a copy** -- the spec lives in
> gitignored `design-specs/`, so no test can ever cross-check against it, and the tracked artifact is
> the only version CI can see. The guard's comment now says plainly that it is a *change* detector
> over a human-maintained count, not a completeness check.
>
> **Verified:** register parses to 14 rows, SPH still the only `to-transfer`; the guard was
> mutation-tested (row removed -> `HasEveryRowTheSpecEnumerates` failed with "Which is: 13 / 14",
> and only that test failed); full suite **818/818** green in `spade/build-ninja/release`.

---

## 3. `SL10`-`SL13` -- the sandbox

`spade_sandbox` is the engine's **reference application**: the place a physics or rendering
change becomes visible rather than inferred from an assertion. It replaces `spade_viewer`
and its v1 bridge entirely. **`RS10a`** described the same executable more narrowly -- "a
minimal window that blits CPU frames from the frame pool, with camera controls, scene
selection and an on-screen frame-rate readout" -- and is subsumed by `SL10`-`SL13`, which
keep the presentation model and add the object-centric UI.

Motivation, from the twenty-fourth spec's section 0: **there is no way to look at the
engine.** The only interactive tool renders v2 physics through the frozen v1 OpenGL engine
across a bridge translation unit that needs `<windows.h>` included first to dodge a macro
collision, and carries its own relaxed warning policy because v1 cannot survive `/W4 /WX`.

### 3.1 `SL10` -- the same-path invariant

**`SL10`** -- the sandbox renders through the **same** `spade::render` entry points the C5
host and the conformance tests use. There is no second render path, no sandbox-only shading,
and no "close enough" preview mode.

Stated as an invariant rather than a preference, and the argument is worth keeping intact:
the sandbox's entire value is that a visual difference implies a real difference. A private
render path makes it a picture of itself, and **a tool that can disagree with the tests is
worse than no tool, because it will be believed.** Guarded by construction -- the sandbox
links `spade::render`, and `SL2b`'s public-surface rule forbids it reaching anywhere else.

### 3.2 `SL11` -- the presenter seam

**`SL11`** -- the CPU rasterizer produces a BGRX8 frame; the sandbox uploads it as a texture
and draws one fullscreen quad; ImGui runs on its OpenGL3 backend. **The window presents a
buffer and hosts a UI -- it is not a renderer.** This sits behind a `Presenter` interface so
S7b substitutes Vulkan presentation without touching a line of sandbox logic.

> 🔴 **TWO DIVERGENCES FROM THIS SIGNED RULING ARE PROPOSED AND NEITHER IS CONFIRMED BY THE USER.
> The ruling's own words above are LEFT EXACTLY AS SIGNED; this note sits beside them.** Recorded
> here rather than settled quietly, because **an absent change looks exactly like an oversight**, and
> the next implementer reads the DESCRIPTOR, not the message thread. Raised by this realm
> 2026-09-18 during a paper design pass for `plan-c-sandbox.md` task `C1`.
>
> **① The identifier `Presenter` -> `TargetSink` (NARROW -- name only, substance untouched).**
> `editor/ui/viewport/presenter.h:84` already declares `class Presenter`, doing very nearly this
> job for the C5 host. A fourth homograph in an estate that has paid for `FramePool` (three
> objects), `component` (two senses) and `sandbox` (two targets). The seam, its purpose and `S7b`'s
> substitutability are unchanged.
>
> **② The backend: this ruling says ImGui "runs on its OpenGL3 backend". ✅ AND THAT IS WHAT IS IN
> FORCE — THIS CLAUSE STANDS AS SIGNED.**
>
> It was superseded to Vulkan on 2026-09-18 and **UNRULED by the user the same day**, after this
> realm withdrew the grounds it had itself supplied (*"Revert to SL11 as signed"*). ⭐ **The clause
> was examined and survived, which is a different and better state than never having been
> questioned** — and this note plus `plan-c-sandbox.md` task `C2` are the only record that it
> happened at all.
>
> ⚠ **The clause's own Vulkan fallback remains what it always was: conditional on the ImGui-loader
> claim failing. That claim was re-verified 2026-09-18 and HOLDS**, so the fallback is not taken and
> nobody should read one into the change history.
>
> 🔴🔴 **BUT THE GROUNDS FOR THAT SUPERSESSION ARE WITHDRAWN BY THIS REALM (2026-09-18) — see
> `plan-c-sandbox.md` task `C2` for the measurement.** In short: the "bare Linux CI runner" cited in
> the argument **has not run since 2026-08-27** (`docker/ci-linux.Dockerfile:4-7`), and **Vulkan does
> not create windows**, so the switch does not avoid the X11/Wayland dependency it was chosen to
> avoid. **The ruling stands as the user's word; its basis does not.** ⭐ **Recommended: revert to
> this clause as signed.** Not enacted — only the user can unrule it. The user was asked with the divergence
> named — *"diverges from your signed text, but for a reason the text does not name"* — so this is a
> **knowing** supersession of their own signed clause, not a general approval read as a specific
> one.
>
> 🔴 **IT IS *NOT* THE FALLBACK THIS RULING DESCRIBES, and that must not be inferred.** The
> paragraph below makes the Vulkan fallback conditional on the ImGui-loader claim failing. **That
> claim was re-verified 2026-09-18 and HOLDS; the trigger did not fire.** Vulkan is taken for the
> `SPADE_BUILD_V1` / Linux-leg reason set out in `plan-c-sandbox.md` task `C2`, which this ruling
> does not name. ⚠⚠ **And the reason matters more than the change, because this ruling
> already contains a Vulkan fallback whose stated trigger DID NOT FIRE.** The paragraph below makes
> the fallback conditional on the ImGui-loader claim failing. **That claim was re-verified on
> 2026-09-18 and it HOLDS** -- the pin is `v1.91.5-docking` and `imgui_impl_glfw.cpp` +
> `imgui_impl_opengl3.cpp` are the vendored backends.
>
> **The real cause is one nobody wrote down: GLFW and ImGui are fetched only inside
> `if(SPADE_BUILD_V1)`** (`spade/vendor/CMakeLists.txt:9-20`, `:42-70`) **because a bare Linux CI
> runner has no X11/Wayland dev packages.** The v2 Linux leg passes *because* it configures
> `V1=OFF`. A GLFW-based window in v2 forces that fetch out of the gate and puts X11/Wayland on the
> only automated leg that proves cross-platform bit-identity -- the leg that computed `a4dec569...`
> and `60e6accc...`. **Losing it does not merely redden CI; it removes the instrument that makes a
> render golden mean anything.**
>
> ⭐⭐⭐ **SO THIS IS A DOCUMENTED FALLBACK BEING TAKEN FOR AN UNDOCUMENTED REASON, AND THAT IS THE
> PART THAT MUST NOT GO UNRECORDED.** Without this note a future reader sees "the Vulkan presenter"
> and correctly infers from the sentence below that the ImGui loader claim failed -- **which is
> false, and was measured false.** *A correct-looking inference from a true sentence is harder to
> catch than a wrong sentence.*

GLAD is **not** reintroduced: ImGui's OpenGL3 backend has bundled its own loader since
v1.87, and the vendored pin is `v1.91.5-docking` (confirmed at `spade/vendor/CMakeLists.txt`,
`GIT_TAG v1.91.5-docking`). The spec requires this be re-verified against the pinned tag at
implementation; if it does not hold, the fallback is the Vulkan presenter using the
already-unconditional Vulkan-Headers and volk, **not** a return to GLAD.

### 3.3 `SL12` -- the sandbox is object-centric

**`SL12`** -- the UI is organised around `SL4`'s object model, "which is what makes it an
engine editor rather than a scene viewer": a scene hierarchy; a per-object inspector listing
attached components with editable configuration; add-object and add-component menus; and an
asset browser over the SDF prefab kit, the parametric mesh library and the material palette.

Live controls: CPU / Vulkan backend, switchable on a running scene and runnable side-by-side
in lockstep so a parity divergence can be **watched** rather than read from a JSON file;
render mode and shading (shaded, wireframe, raymarch, velocity per `SL9c`, and a
raster-vs-raymarch difference view); sun direction, sky, exposure, shadows and per-object
material; gravity, timestep, substep count and contact parameters.

Inspection: physics debug draw -- contact points and normals, body AABBs, the broadphase
grid, force and velocity vectors -- extending the existing depth-biased overlay path rather
than adding one; pause, single-step and scrub with snapshot save and restore on the existing
`kSnapshotVersion` machinery; SDF slice and distance-field visualisation.

> **RECONCILED:** `RS10c` and `SL11`/`SL12` are about **different ImGui surfaces**.
>
> `RS10c` calls the ImGui world/scene builder "a named seat, not built here" -- "This
> program does not build it, and must not foreclose it" -- and then places the HUD: "The
> ImGui HUD itself arrives with S7b, where the long-linked-never-used Dear ImGui dependency
> finally earns its place." But `SL11`, `SL12` and `SL17`'s Phase P5 do build an ImGui shell
> -- "presenter, ImGui shell, hierarchy and inspector, asset browser, live controls,
> inspection surfaces, headless mode".
>
> **Resolution is by surface, and both stand:**
>
> - The **sandbox's ImGui shell** (hierarchy, inspector, asset browser, live controls) is
>   `SL11`/`SL12`'s, built in Plan C. It is `RS10c`'s named seat being taken by the later
>   spec -- exactly the "must not foreclose it" outcome, arriving sooner than `RS10c`
>   expected. Plan C is now **written** (`plan-c-sandbox.md`, 2026-09-17) and **has not been
>   executed**: no `spade_sandbox` target exists. The distinction matters -- a written plan moves
>   the seat from unspecified to scheduled, and nothing else.
> - The **ImGui debug HUD on the GPU path** stays S7b, unchanged. The twenty-fourth spec's
>   own exclusion list confirms it: "Vulkan presentation, camera sensors, the ImGui debug HUD
>   on the GPU path. S7b's, behind `SL11`'s Presenter seam."
>
> The two are not the same artifact and never were; `RS10c` reads as a conflict only because
> it uses "ImGui" for both.

### 3.4 `SL13` -- scene sources and persistence

**`SL13`** -- three sources behind one picker: curated presets (including section 4's
successors to the retired scenes); test scenarios, surfaced in a dev mode through the
existing `ScenarioData` loader so any failing case can be opened and watched; and saved
sandbox scenes.

The sandbox saves its own scene format (JSON; `nlohmann/json` is already a dependency),
screenshots, and physics snapshots. It does **not** write `.world.yaml` or Kat recipes.
Canonical content authoring stays where it is -- "a second authoring home for production
content is exactly the outcome this clause exists to prevent."

---

## 4. `SL14` -- v1 quarantine and scene equivalence

### 4.1 `SL14a` -- quarantine, not deletion

**`SL14a`** -- `spade/src/`, `spade/include/`, `spade/examples/`, `spade/assets/`,
`engine/tools/viewer/` and the `SPADE_BUILD_V1` option **move to `spade/legacy/` as pure
renames with zero edits**, following the repo's existing quarantine precedent exactly. GLFW
and ImGui move from the `SPADE_BUILD_V1` gate to `SPADE_BUILD_SANDBOX`; **GLAD leaves the
build entirely.** See the SUPERSEDED block in section 0: this clause replaces `RS10b`'s
deletion.

**Location: inside `spade/`, not the repo-root `legacy/`.** Root `legacy/` is a Kat concept,
and Spade's own history belongs to Spade -- "`SL1`'s identity claim would be hollow if the
engine's predecessor were archived as a Kat artifact."

**"Completely cut off" is enforced, not asserted:** no build wiring references it, no target
links it, no test includes it, and a ratchet guard in the shape of the existing
legacy-quarantine test fails if anything reaches back in. `SL18` requires the ratchet be
demonstrated by a seeded violation, not merely written.

**One thing must be harvested before the viewer moves.** `engine/tools/viewer/scenes.cpp` is
**v2 code, not v1** -- only `bridge.cpp` touches the old engine. Its eight scene
constructors are the direct source material for the eight successors in `SL14b` and are
**ported forward, not re-derived from screenshots.**

### 4.2 `SL14b` -- scene equivalence, and the order it must happen in

**`SL14b`** -- the retired scenes need successor presets that exercise the same objects,
physics and rendering -- **not numerically exact reproductions** (user ruling 8, 2026-08-30).
Equivalence is judged on three axes, and the axes are **not uniform across the eleven
scenes**, so each is stated honestly rather than promising one comparison for all of them.

**The binding constraint is ordering.** Baselines can only be captured while v1 still
builds. Once it is quarantined and cut from the build it can never be run again, and the
comparison becomes impossible forever:

```
characterise what each scene exercises
  -> capture baselines from the LIVE v1 tools   <- last moment this is possible
  -> build the successor presets
  -> compare on all three axes
  -> only then quarantine
```

**The two populations.** The eleven scenes are not one set. The **eight `spade_viewer`
scenes** -- drop, bounce, shower, gate, hover, wind, flight, swarm -- already run **v2
physics** and are merely rendered through the v1 bridge. The **three `Sandbox.exe` scenes**
-- fluid (50,000-sphere SPH), spheres, cubes -- run **v1's own engine**, which is the thing
being retired.

| Axis | The eight v2 scenes | The three v1 scenes |
|---|---|---|
| Functional | Bit-identical state trajectories asserted across the port. The engine underneath is unchanged, so this is available, and it is far stronger than visual judgement. | Capability assertion: the successor exercises the named systems (SPH, grid collision, contact saturation) and runs stably. **No trajectory comparison exists** -- the engine that produced them is being retired. |
| Visual | Reference frames captured from the live tools before quarantine; successors judged equivalent on geometry, motion and materials. **Not a pixel diff** -- v1 rasterises on the GPU through OpenGL and the sandbox blits CPU frames, so pixel equality is impossible and asserting it would be a fake test. The new frames are then pinned as goldens, so everything after this program is diffable. | Same treatment. |
| Latency and memory | Physics step time and memory compared directly via `spade_bench`, which already benchmarks `Simulation::step()` and carries a recorded `baselines.json`. | Fresh baselines recorded only. |

**Render time is not comparable on either population.** CPU rasterisation is slower than
hardware GL by design; recording that as a regression would be wrong. It is recorded as a
new baseline for the CPU path.

### 4.3 `SL14c` -- fluid becomes a standing capability target

**`SL14c`** -- the 50,000-body scene is the largest stress case in the tree and is retained
as a **permanent preset and a stated sandbox requirement**, not merely as a migration
artifact. The three v1 successors are labelled as **capability demonstrations, never as
parity or regression claims**, so that no later reader mistakes them for a numeric guarantee
they cannot support.

---

## 5. `SL15` -- build, CI, and how a windowed tool gets tested

> ✅ **AMENDED BY USER RULING, 2026-09-18 — `SL15a` IS NOW "DEFAULT ON, BACKEND-DETECTED", AND THE
> "WHICH CI CONFIGURES OFF" CLAUSE IS STRUCK.**
>
> **What is in force:** `SPADE_BUILD_SANDBOX` **defaults ON everywhere including CI**; the windowing
> backend is **probed, not assumed** (`find_package(X11 QUIET)` drives `GLFW_BUILD_X11`, and
> `SPADE_GLFW_HAS_BACKEND` is cached so the sandbox can refuse at runtime with the real reason).
>
> ⚠ **THE ORIGINAL CLAUSE IS LEFT STANDING BELOW, STRUCK RATHER THAN DELETED**, and the annotation
> after it stays as the provenance. ⭐ **The measurement is why the ruling was asked for; the ruling
> is why the clause changed. Deleting either leaves the next reader with a text that was never
> questioned.**
>
> **AUTHORITY, STATED PRECISELY RATHER THAN SUMMARISED.** The user's own words, relayed through the
> coordinator: *"Proceed with the recommendations, properly updating the decision in the records and
> to the realms with the instruction to record and hold, then push to origin and hold."*
> ⚠ **The user ruled on the RECOMMENDATION; the amendment's wording is the coordinator's, adopted.**
> Recorded at the phase plan §74. `L302` discharges on this ruling.
>
> ⛔ **What this does NOT license:** it does not retire the measurement, and it does not make the
> first Linux CI run predicted rather than observed -- **nothing had been pushed when the probes ran,
> so CI has still never configured the sandbox.** See the annotation below.

~~**`SL15a`** -- **build gating.** A new `SPADE_BUILD_SANDBOX` option, **default ON, which CI
configures OFF**~~ -- the same pattern `SPADE_BUILD_V1` uses today and that CI already
exercises. The reason the windowing dependencies were gated in the first place is recorded
in the vendor file and still applies: **a bare Linux runner has no X11/Wayland development
packages, and `FetchContent_MakeAvailable(glfw)` fails its own configure without them.** The
option is not a preference, it is a property of the runner.

> ⚠⚠ **ANNOTATION, 2026-09-18 -- FACTS BESIDE THE SIGNED WORDS. THE CLAUSE ABOVE IS UNCHANGED AND
> THIS NOTE PROPOSES NO REMEDY.** The signed text stands as written; what follows is measurement.
> ⭐ **A clause whose premise is false is still the user's clause**, and what `SL15a` should now say
> is theirs to decide, not this realm's and not the coordinator's. Four facts, nothing else:
>
> **① THE CLAUSE'S STATED REASON WAS MEASURED FALSE, and by the realm that wrote the clause's
> subject.** The reason given is *"a bare Linux runner has no X11/Wayland development packages, and
> `FetchContent_MakeAvailable(glfw)` fails its own configure without them… not a preference, it is a
> property of the runner."* **Measured 2026-09-18 on the real CI image** (`kat-ci-linux:local`,
> X11=no GL=no EGL=no), **three probes each paired with a control that had to fail: GLFW configures
> and builds with `GLFW_BUILD_X11=OFF GLFW_BUILD_WAYLAND=OFF`, and the control with X11=ON failed at
> `find_package`, which is what proves the probe could see the absence.** The fix landed at
> `b37508d8`: `find_package(X11 QUIET)` drives the backend, `SPADE_GLFW_HAS_BACKEND` is cached, and
> the sandbox is told either way so it can refuse with the real reason. ⭐ **A configure failure
> under a DEFAULT was read as a dependency on the PLATFORM** -- it was a dependency on a default,
> and the default is ours to set.
>
> **② CI DOES NOT IMPLEMENT THE `OFF`, AND NEVER HAS.** `SPADE_BUILD_SANDBOX` occurs **nowhere in
> `.github/`** (unbounded grep, no match), and the option is declared `ON` in `spade/CMakeLists.txt`.
> **So CI takes the default, which is ON -- the opposite of the signed clause.**
>
> **③ `SL15a` AND `SL15b` CANNOT BOTH HOLD.** `SL15b` requires that `spade_sandbox --headless`
> *"runs in CI"*. ⚠ **If CI configured `SPADE_BUILD_SANDBOX=OFF` there would be no `spade_sandbox`
> target at all**, so the headless test surface could not run. **The implementation satisfies `SL15b`
> by violating `SL15a`.**
>
> **④ WHAT IS IN FORCE: ON, WITH BACKEND DETECTION** -- the sandbox target is built, the windowing
> backend is probed rather than assumed, and a runner without one gets a target that compiles and
> links and refuses at runtime with the real reason.
>
> ⚠ **AND A LIMIT ON ALL FOUR, STATED SO NOBODY READS THEM AS A CI OBSERVATION:** `origin/master` is
> `9603d4c6` and **neither `ac18c480` nor `b37508d8` is an ancestor of it.** ⭐⭐⭐ **NOTHING IS
> PUSHED, SO CI HAS NEVER SEEN THIS CODE -- the gate has never once configured the sandbox, in
> either direction.** Every sentence above about what CI *does* is a property of **`master`**, and
> *a property of `master` is not a property of `origin/master`.* **Recorded in the ledger as
> `L302`.**

**`SL15b`** -- **headless is the test surface.** `spade_sandbox --headless` loads a scene,
applies options, steps, renders and dumps frames with **no window**, feeding the existing
frame-dump and golden-image pattern. **Every panel's logic is reachable from it**, and it
runs in CI with the window build off.

The window itself is a **deliberately dumb shell**: presentation and input, nothing else.
**Any logic that migrates into it is a defect, because it becomes untestable by
construction.** This is the concrete answer to "how does a GUI tool avoid regressing
silently" -- by having almost nothing in the GUI.

Environment note: hosted CI was failing on a **billing block rather than a quota** when the
spec was drafted, so every claim in the program was proven on local evidence. That is a
condition of the environment, not of this design. ~~The block has since been resolved;~~ the
discipline it forced is worth keeping, because local-green is a claim about one toolchain.

> 🔴🔴 **CORRECTION 2026-09-18 — THAT SENTENCE WAS TRUE WHEN WRITTEN AND IS FALSE NOW, AND THE
> WAY IT WENT FALSE IS THE FINDING.** Hosted CI **did** recover. Boundaries, each measured against
> the ADJACENT run rather than inferred: last job with no runner **2026-08-31T12:55:55**, first job
> with a runner **2026-09-01T11:14:29**, last job with a runner **2026-09-07T19:13:11**, first job
> with none again **2026-09-08T05:19:29**. ⚠ **Say the unit or the number is not a measurement:**
> that is **6 d 8 h elapsed** and **7 calendar days (09-01 … 09-07)** — both true, of different
> things. **Since the close it is EXHAUSTIVE, not sampled: all 143 runs checked, 143 with no runner
> assigned, zero exceptions** — every job completing in ~2 s with `runner_name` empty and **zero
> steps**, against live jobs of 5/9/13/14/18/24 steps on named runners before it.
>
> 🔴 **AN EARLIER VERSION OF THIS NOTE DATED THE RECOVERY 2026-09-02T21:51 AND IT WAS WRONG, BY THE
> DEFECT THIS NOTE EXISTS TO WARN ABOUT.** I dated the opening boundary by the run's `conclusion` —
> **the one field that cannot tell "tests failed" from "job never started"** — so a week of runs
> where runners WERE assigned and tests merely failed read to me as outage. ⭐⭐⭐ **I APPLIED THE
> RIGHT INSTRUMENT TO THE SIDE I SUSPECTED AND THE WRONG ONE TO THE SIDE I DID NOT: SUSPICION, NOT
> KNOWLEDGE, SELECTED THE INSTRUMENT.** ⚠ *A rule stated in prose does not bind the next
> measurement; only an instrument does.*
> ⚠ **The cause is NOT measured here.** What is measured is *no runner assigned*; GitHub's own text
> on the blocked jobs names a payments/spending-limit condition, which is a strong witness and not a
> reading of the billing page.
>
> ⛔ **THIS NOTE DELIBERATELY STATES NO STATUS, BECAUSE A STATUS IS WHAT ROTTED.** The rule that
> governs instead, and it holds in both directions: ⭐⭐⭐ ***ANY CLAIM ABOUT HOSTED CI CARRIES THE
> DATE IT WAS MEASURED, OR IT IS NOT A CLAIM.*** What *"CI passed"* means in this repo is the
> runners in `docker/`, and that sentence is a condition rather than a moment.
> ⭐⭐ **When billing is restored the first run is an UNKNOWN, never an expected green** — it will be
> the first execution of everything landed since the boundary, so its red implicates nothing in
> particular until `runner_name` and the step count have been read.

---

## 6. Sequencing and the gate

**`SL17` in full.** ⭐ **HARVESTED COMPLETE 2026-09-17**, so the ruling no longer lives partly in a
superseded file. This table previously carried **five of `SL17`'s eight rows** -- P1, P2 and P3 were
omitted because they are *done*, which is defensible per row and misleading as a table: it read as
though the programme had only ever had five phases.

**Entry condition** (`SL17`, verbatim): *"the 21st spec's render program completes C1 → C2 → C3 →
C4 → C5 and merges to master. C4 is slotted after C3 by user ruling, discharging `SR-42`'s 'C0
cannot merge without C4' and retiring the render branch's merge debt before this program adds to
it."* ⚠ `SR-42` is one of the 29 orphaned `SR-nn` (`07-status.md` s.6) -- **cited here as `SL17`
cites it, not reconstructed**, per the user's as-touched ruling.

| Phase | Contents | Exit | State |
|---|---|---|---|
| P0 -- Audit and baselines | Characterise all 11 scenes; capture **visual, functional and performance** baselines from the **live** v1 tools; ratify the `SL7` register against the tree | Baselines recorded and reviewed. **Nothing may be quarantined before this completes.** | ⏸ |
| P1 -- Object model | `SL3`-`SL5`: object graph, component attachment, registered type ids, serialization, the component types | Snapshot and parity estate provably unchanged; object graph proven determinism-neutral | ✅ Plan A, `9ed536f6` |
| P2 -- Behaviors | `SL6`: declared-pass registration, ordering, read/write declarations, GPU-eligibility refusal; the kinematic mover as the first behavior | A CPU-only behavior is **refused** from a GPU-authoritative world, proven by construction | ✅ Plan A, `9ed536f6` |
| P3 -- Transfer gaps | `SL9c`-`SL9e`: velocity render mode, camera and input components, instancing helpers | Register rows closed; the three smaller gaps demonstrable | ✅ Plan A, `9ed536f6` |
| P4 -- SPH fluid | `SL8`: CPU and GPU passes, grid reuse, `Fluid` component, measured parity band | 👤 **CHECKPOINT ①** -- fluid runs on both backends inside its measured band, **or is honestly declared CPU-only** | 🔴 Plan B unwritten |
| P5 -- Sandbox | `SL10`-`SL13`: presenter, ImGui shell, hierarchy and inspector, asset browser, live controls, inspection surfaces, headless mode | 👤 **CHECKPOINT ②** -- the sandbox opens a test scenario, a preset and a saved scene; **every control operates live** | ⏸ Plan C **written** `plan-c-sandbox.md`, unexecuted |
| P6 -- Successors | `SL14b`: the 11 successor presets; three-axis comparison against P0's baselines | 👤 **CHECKPOINT ③** -- equivalence demonstrated on all three axes, **with the honest asymmetries stated** | 🔴 |
| P7 -- Quarantine and hardening | `SL14a` quarantine + ratchet guard; `SL2a` release identity; `SL2b` public-surface guard; **`SL16` vocabulary note** | 👤 **CHECKPOINT ④** -- v1 quarantined and unreachable; standalone build → install → consume green with **zero GL dependency** | 🔴 |

> ⚠⚠ **P7 HAD ALREADY LOST A DELIVERABLE, WHICH IS WHY A PARTIAL HARVEST IS NOT A SMALL
> COMPROMISE.** The live row read *"`SL14a` quarantine plus ratchet guard; `SL2a`/`SL2b`
> hardening"* -- collapsing two named deliverables into the word "hardening" and **dropping
> `SL16`'s vocabulary note entirely.** Nothing in any surviving word was wrong. A deliverable
> simply stopped being listed, in the only table that ever listed it, and no check could see it
> because **a shortened row is not a malformed one.**
>
> **The four checkpoint numbers went the same way.** ①-④ are how the *user* is asked to sign, and
> the live rows had reduced them to an unnumbered "Checkpoint". A summary that drops the thing
> the reader is supposed to act on keeps every fact and loses the point.

**Current state.** Plan A has landed on master and closed the `SL9c`/`SL9d`/`SL9e` rows.
Plan B (`SL8`, P4) is unwritten. Plan C (the sandbox, P5) is **written** (`plan-c-sandbox.md`,
2026-09-17) and unexecuted. P6 and P7 cannot start before
them, and P7 additionally cannot start before the register reaches zero open rows -- the
BLOCKED condition in section 0, and the one thing here a reader must not work around.

---

## 7. `SL18` -- verification, the complete nine

⭐ **HARVESTED COMPLETE 2026-09-17.** `SL18`'s nine obligations previously existed in the live spec
set only as a **five-item subset** in `04-objects.md` s.9, correctly scoped there to *"the ones this
document's subject owns"*. That subset is right and stays. **What did not exist anywhere live was
the whole list** -- so the four obligations belonging to no single document belonged to no document
at all, and a reader could satisfy every list they could find while leaving `SL18` unmet.

**The nine, verbatim from `SL18`, with the owner named for each** -- the column exists so a future
partial restatement is a visibly incomplete table rather than a plausible short one:

| # | Obligation (`SL18`'s own words) | Owner | State |
|---|---|---|---|
| 1 | *"The object graph does not perturb physics -- an existing golden scenario produces bit-identical results with and without an object graph built over it. This is the `SL3` claim, tested rather than argued."* | `04-objects.md` | ✅ **Discharged**, `test_objects_determinism.cpp` |
| 2 | *"Snapshot compatibility -- a snapshot taken before P1 restores after it, unmodified."* | `04-objects.md` | ✅ Plan A |
| 3 | *"Behavior determinism -- a world with a behavior produces identical trajectories across runs, processes and backends; a CPU-only behavior is refused from the GPU path rather than silently degraded."* | `04-objects.md` | ⚠ **Partly unobservable** -- the *across-backends* half is asserted by the GPU battery, which **no CI job has ever run** (`07-status.md` rule 14). The refusal half is proven by construction. |
| 4 | *"SPH parity -- measured band with recorded provenance, never widened to accommodate a result."* | `SL8` / this doc | 🔴 Plan B unwritten |
| 5 | *"Transfer completeness -- a guard asserts every `SL7` register row is dispositioned, so a future v1 system cannot be quarantined unnoticed."* | this doc | ✅ `spade/tests/test_transfer_register.cpp`, 13 rows |
| 6 | *"Public-surface purity -- the sandbox's include graph touches only installed headers (`SL2b`)."* | this doc | ⏸ Plan C -- **the guard has no subject until the sandbox exists** |
| 7 | *"Standalone consumption -- the out-of-tree `find_package(spade)` consumer builds and runs with `SPADE_BUILD_SANDBOX=OFF` and no GL anywhere."* | `01-charter.md` / this doc | ✅ for the consumer; ⏸ for the flag, which does not exist yet |
| 8 | *"Quarantine ratchet -- a seeded violation reaching into `spade/legacy/` fails the guard."* | this doc | ⏸ blocked with the quarantine itself (`SL14a`) |
| 9 | *"Scene equivalence -- the eight v2 successors assert bit-identical trajectories; all eleven carry pinned golden frames going forward."* | `SL14b` / this doc | 🔴 Plan C |

**Guard discipline, carried from the render program** (`SL18`, verbatim):

> *"A constant, test, or fixture validated only against something that cannot vary is unvalidated.
> That lesson was learned six times over in the 21st spec's execution. **Every guard this spec adds
> must be shown to fail under the mutation it exists to catch** -- the transfer-completeness guard
> with a row removed, the quarantine ratchet with a seeded reach-back, the purity guard with an
> internal include, the equivalence assertions with a perturbed trajectory."*

⚠ **Obligation 3 is the one to read twice before signing.** It is the only row whose subject is
*measured by a suite that cannot fail where it runs.* Nothing about the obligation is wrong; what is
wrong is that reading this list would previously have left you believing cross-backend determinism
is checked. It is checked on a developer's machine, by hand, or not at all. That is now declared in
`07-status.md` rule 14 and at the head of `spade/tests/CMakeLists.txt`, and it is stated **here**
because this is the list a signer reads.

---

## 8. `SL16` -- vocabulary

⭐ **HARVESTED 2026-09-17.** `SL16` had **no definition anywhere in the live spec set** -- it was
cited by name in `SL17`'s P7 row and nowhere stated. Verbatim:

> **`SL16` -- "component" is shared and context-scoped** (user ruling, 2026-08-30). *"Both Spade
> and Kat keep the word **component**. They are the same idea -- a part attached to an underlying
> structure -- and the trees are worked separately, so context disambiguates in practice.*
>
> *Two things make that safe rather than merely convenient. In code the namespaces already separate
> them: Spade's are C++ types under `spade::`; Kat's are C3-contract manifests addressed by dotted
> `typeId` strings such as `decision.policy`. And `AGENTS.md`'s vocabulary section records that
> both senses are live and context-scoped, so a reader meeting the word at the `dronesim/spade/`
> boundary -- the one place both appear in a single file -- knows there are two.*
>
> *Not renamed: `element` stays the force-element layer's word (rotors, drag surfaces) in Spade,
> and is not a synonym for a component in either tree."*

### 🔴 `SL16` rests on two legs and **one of them does not hold today**

Measured 2026-09-17, because a harvested ruling is worth checking against the tree rather than
transcribed into it:

* ✅ **Leg 1 -- the namespaces do separate them.** Verified positively: Kat's are dotted `typeId`
  strings in C3 manifests (`components/command.mixer/manifest.json` -> `typeId: command.mixer`);
  Spade's is the C++ `enum class ComponentTypeId` (`spade/engine/objects/component.hpp:31`). And
  `SL16`'s premise about the boundary is real -- **four files under `dronesim/spade/` mention the
  word**, which is exactly the "one place both appear" it names.
* ✅ **Leg 2 -- NOW HOLDS. It did not on the morning of 2026-09-17.** `AGENTS.md` **did** have a
  section headed **"Naming vocabulary"** (line 63) -- so the citation *resolved*, which was the
  whole problem -- but that section is about **docs pages using domain vocabulary rather than
  directory names** (`observation/` → estimation, `perception/` → vision). It said nothing about
  `component`. **`AGENTS.md` now carries a "Words with two live senses" subsection** stating both
  senses with their addresses and that `element` is not a synonym in either tree. Leg 2 is
  satisfied by that entry and by nothing earlier.

⚠⚠ **A LIVE DECOY, AND `SL16` IS ITS OWN VICTIM.** A reader checking the claim finds a section with
the right *name* about the wrong *subject*, and a section that answers is far worse than one that is
missing -- the missing one sends you looking. This is the instrument/question vocabulary mismatch
`docs/dev/one-name-two-objects.md` names, arriving inside a ruling **about** a name having two
meanings.

**And the tense is the other half of it.** `SL17`'s P7 lists *"`SL16` vocabulary note"* as a **P7
deliverable** -- something to be **written**. `SL16` describes it in the present tense, as something
that already exists. **A ruling that states its own deliverable as an accomplished fact removes the
only signal that it is outstanding**; P7 has never run, so the note was never written, and `SL16`
has read as satisfied since the day it was drafted.

✅ **Disposition: DISCHARGED 2026-09-17.** The ruling stands on both legs. The `AGENTS.md` note --
`SL17`'s P7 deliverable, arriving early rather than as new scope -- is written, and **`SL16` is the
first P7 line item complete.** It was routed through the coordinator before being taken, because
`AGENTS.md` is estate-wide and not this realm's.

> ⭐⭐ **THE TENSE IS WHY THIS SAT UNDONE FOR EIGHTEEN DAYS, AND IT IS THE TRANSFERABLE PART.**
> `SL16` describes its own deliverable **in the present tense**, as a note that already exists.
> `SL17`'s P7 lists the same note as **something to be written**. Both sentences are about the same
> object and only one of them is a claim about the world. **A ruling that states its own
> deliverable as an accomplished fact removes the only signal that it is outstanding** -- there is
> no open checkbox, no "owed" marker, nothing for a status sweep to catch, because the ruling
> reads as its own completion certificate.
>
> **And the decoy made it unfalsifiable by the obvious check.** A reader who did go and look found
> a section headed exactly *"Naming vocabulary"* and stopped. **A dangling reference would have
> been caught years ago; a resolving one that is about something else survives every link checker
> this estate owns.** When a ruling cites a document as evidence for itself, check what the cited
> section *says*, not that it *resolves*.

---

## 9. Marker index

| Marker | Subject | Where |
|---|---|---|
| **SUPERSEDED** | `RS10b`'s "deleted" is dead text; `SL14a`'s quarantine is operative. Includes the `tools/viewer/` vs `engine/tools/viewer/` path disagreement, resolved against the tree in SL's favour. | Section 0, restated at 4.1 |
| **BLOCKED** | The quarantine must not execute: one `to-transfer` row remains -- **SPH fluid**. | Section 0, restated at 6 |
| **RECONCILED** | `RS10c`'s "ImGui HUD arrives with S7b" vs `SL11`/`SL12`'s ImGui shell. Different surfaces: the sandbox shell is SL's (Plan C written 2026-09-17, unexecuted); the GPU-path debug HUD stays S7b. | Section 3.3 |
| **FIXED 2026-09-07** | `SL9f` (barycentric wireframe) was absent from the as-built register, whose guard pins 13 rows. | Section 2.3 |
