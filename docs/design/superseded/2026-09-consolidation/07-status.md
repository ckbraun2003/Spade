# Spade -- status: specified vs built

**What this document is.** The single live scorecard for the Spade estate: one row per spec
obligation, each naming its evidence. It is the only document in `design-specs/spade/` that
describes *the present*; the other eight describe *the design*. When it disagrees with a spec,
the spec states the intent and this file states what exists.

**What it replaces.** Consolidation ruling **`CN-6`** ("two Spade as-built records -> one live
scorecard"), executed 2026-09-07. It harvests and supersedes both:

- `superseded/kat-spade-execution-record.md` -- the running scorecard, last updated 2026-08-17
- `superseded/kat-spade-s1-s4-implementation-record.html` -- the frozen S1-S4 narrative, 2026-08-11

Both are in `superseded/`. Nothing was dropped: the S1-S4 record's forward-obligation table is
merged into section 6 below, and its per-task traceability into section 7.

**Update discipline.** Append-only per phase. Each close-out edits sections 1-3, adds its
evidence, and moves discharged rows out of section 6 into the changelog entry that closed them.

> **How to read a test count in this file.** A ctest tally is a property of **a build tree**,
> never of a commit -- three trees on this box have reported different totals for the same
> source with the same flags. Every number below carries the tree and the commit it was measured
> in. A number without that provenance is not evidence.

---

## 0 - State as of 2026-09-07 (verified against git, not recalled)

| Fact | Value | How verified |
|---|---|---|
| master | `9a9fd34b` | `git log -1` |
| origin/master | `a92218d4` -- **master is 1 commit ahead, UNPUSHED** | `git rev-list --left-right --count` |
| Substep passes | **10** (`kSubstepPassCount = 10`) | `spade/engine/physics/schedule.hpp:282` |
| Component types | **10 declared**, ids `transform`=0 .. `fluid`=9 | `spade/engine/objects/component.hpp:31-44` |
| `FluidComponent` | declared with fields, **no solver** -- "Populated by Plan B (SL8)" | `component.hpp:90-97` |
| v1 quarantine | **NOT executed.** `spade/src`, `spade/include`, `spade/examples`, `spade/assets` all present; `spade/legacy` absent; `SPADE_BUILD_V1` still an option, default `ON` | filesystem + `spade/CMakeLists.txt:38` |
| Transfer register | **one open row: SPH fluid** | `spade/docs/v1-transfer-register.md` |

Phases landed, each confirmed an ancestor of `HEAD` by `git merge-base --is-ancestor`:

| Phase | Commit | Date | What |
|---|---|---|---|
| S1-S4 | `6772f0c` | 2026-08-11 | v2 engine, M1B bar met (PR #5) |
| S5 | `6bc6b5c` | 2026-08-13 | corpus-is-data (PR #13) |
| S6 | `153275f` | 2026-08-17 | Vulkan compute backend + CPU/GPU parity (PR #15) |
| S7a | `9778260` | 2026-09-02 | rendering and scene representation |
| (CI) | `e6e3082` | 2026-09-03 | first hosted-CI run green (PR #16) |
| SL Plan A | `9ed536f6` | 2026-09-07 | object model, behaviors, transfer register |
| (tooling) | `9a9fd34b` | 2026-09-07 | `specs` check group removed by user ruling; `spec_corpus()` kept, recursive, excluding `superseded/` -- **unpushed** |

---

## 1 - Decision record `D1`-`D12` -- implementation status

| # | Decision | Status | Evidence |
|---|---|---|---|
| **`D1`** | Symplectic Euler + quaternion exp-map, substeps, integrator = pure fn | BUILT (S3) | `physics/integrator.*`, `core/math_ops.hpp`; energy + ballistic validation suites |
| **`D2`** | fp32 both paths, pinned reduction order, local-frame worlds | BUILT -- and **strengthened beyond spec** | All libm transcendentals retired to in-engine `core/fp32_math` kernels (`log32`/`sin32`/`cos32`/`exp32`, <=1 ulp exhaustively verified) after a real 1-ulp MSVC-vs-glibc parity break. Cross-platform bit-identity is a proven contract, CI-enforced |
| **`D3`** | Analytic SDF static worlds + sorted-grid dynamic-dynamic; mesh colliders later | BUILT (S3) | `world/sdf.*` (primitives, CSG, heightfield, flat postfix program), `physics/` grid |
| **`D4`** | Model-type layer, Quadrotor first | BUILT (S4); carried gap **closed** (S6 T2) | `vehicles/model_type.*`, `vehicles/quadrotor.*`. `BodyState.proxy_radius` with `effective_proxy_radius()` as the one canonical predicate, consumed by both collision passes and the GPU kernels |
| **`D5`** | Rotor = curves + RPM lag + momentum-theory inflow + SDF ground effect; BEMT = `P7` roadmap | BUILT (S4) | `vehicles/rotor.*`; climb/descent vs momentum-theory reference curves |
| **`D6`** | Medium interface; Dryden seeded turbulence | BUILT (S4) | `world/medium.*`; Dryden spectral sanity + assembled-recursion bias law verified at operating theta |
| **`D7`** | WorldBuilder + versioned YAML world file; glTF later, render-only | BUILT (S5); **schema lifted v1 -> v2** by `RS5` (S7a) | `world/builder.*`, `world/world_file.*`. v1 was user-frozen 2026-08-12; v2 added materials, lighting and props. See `03-world-and-render.md` |
| **`D8`** | Native world-index batching everywhere; N=1 degenerate; CPU twin serial | BUILT -- both halves (S6 closed the GPU half) | CPU: `(world_id, cell)` grid keys, per-world capacities, world-partitioned SoA arenas. GPU: batched sorted-grid collision, per-world config buffers, batching-invariance bit-identical solo vs in-set |
| **`D9`** | Slang single-source layouts -> generated C++ headers + binding registry | BUILT (S6) | `cmake/SpadeSlang.cmake` + generator: slangc reflection -> `layout_check.gen.hpp` (236+ per-field static_asserts) + `bindings.gen.hpp`; 28 SPIR-V modules machine-scanned by rules `P1`-`P5`, `E1`/`E2` |
| **`D10`** | Strangler: v1 GL + sandbox untouched until the renderer replaces them | **HELD -- and still held today.** v1 is present and untouched | Verified 2026-09-07 (section 0). The endpoint moved twice: `RS1` reassigned S7's exit proof to a CPU rasterizer, and `SL14a` changed the disposition from delete to quarantine. See `06-sandbox-and-v1.md` |
| **`D11`** | GTest/CTest from first commit; determinism + parity spine; own CI; bench in-repo | BUILT -- parity spine complete (S6) | CPU/GPU parity over the full committed corpus, bands measured-then-pinned with device provenance; three-grade determinism honored; invariance battery discriminating |
| **`D12`** | MSVC + Ninja + presets, C++23, `std::expected` spine | BUILT (S1) | `CMakePresets.json`; `Result<T>`/`spade::Error` throughout; no exceptions cross the boundary |

> **Note on `D11` and the schedule.** `D11`'s parity spine was certified against an **eight**-pass
> schedule. The schedule is now ten (`SL6`). The certification survives because both new slots are
> inert with no behavior registry attached -- which is every golden-corpus scenario, so the corpus
> re-proves that inertness on every run rather than it having been a one-time observation. This is
> the reasoning, not a hand-wave; see `02-engine.md`.

---

## 2 - Phases `S1`-`S8` -- exit proofs

| Stage | Spec exit proof | Status |
|---|---|---|
| **`S1`** | CI green on math unit suite; bench baseline emitted | DONE. `spade::` install/export landed, hardened in S5 by the out-of-tree consumer smoke (which caught a real `EXPORT_NAME` defect) |
| **`S2`** | Lifecycle + snapshot round-trip green | DONE. Registry-walk snapshot blobs; restore byte-faithful. The S1-era `ecs/` scaffold was superseded by `state/` arenas and deleted in S5 (`D-S5-1`) -- physics never ran on it |
| **`S3`** | Determinism replay green; ballistic/energy green | DONE. Golden corpus born here; digest = FNV-1a fold over the registration walk |
| **`S4`** | Hover-trim + climb/descent green; **charter M1B bar (headless CPU)** | DONE -- **M1B MET**. `test_m1b_bar.cpp` charter-bullet asserts + `A3` conformance cases. Rate targets hit on CPU: ~296k substeps/s single-world (30x the 10 kHz floor) |
| **`S5`** | Corpus is data; C5 `world_ref` loadable | DONE. Five `.scenario.yaml` + seven worlds; `.digest` files retired with all four values carried byte-identical; builder-equals-data proven by a committed bridging test with an independent hard-coded second source |
| **`S6`** | Parity bands green; batching-invariance green; bench sweep recorded | DONE 2026-08-17. Parity green over the full corpus (worst rel 8.27e-5; `bounce` and contact-pair rows **bit-identical**; every integer lane bit-exact). `fp32_math` Slang ports bit-exact on device over 99,734,618 args, 0 mismatches. Knob-invariance sweep **discriminating** (mutation-killed) |
| **`S7a`** | Scene model, CPU rasterizer, frame pool, sandbox, v1 retirement | **DONE 2026-09-02** (`9778260`), CI green 2026-09-03 (`e6e3082`). Split from `S7` by `RS1`. **The v1-retirement half did NOT ship** -- see section 6 |
| **`S7b`** | Vulkan render backend, ImGui debug HUD, camera-sensor seat | NOT STARTED |
| **`S8`** | Effects demos under Vulkan; promotion rule documented | NOT STARTED |

> **`S7`'s exit proof was reassigned.** The approved engine design names a *Vulkan* renderer as the
> thing that replaces v1. `RS1` split S7 and declared the original exit proof met by S7a, which
> ships a CPU rasterizer and no Vulkan at all. That amendment lives in a spec that is still a
> draft. Recorded in `RECONCILIATION.md` as `C4`.

---

## 3 - Programs beyond the S-series

| Program | Spec | Status |
|---|---|---|
| Rendering and scene representation | 21st, `RS1`-`RS15` | Program complete and merged (S7a). ✅ **SIGNED 2026-09-17** -- see section 3.1. ⛔ **`RS10b` EXCLUDED from the signature.** |
| Spade as a library: objects, behaviors, sandbox | 24th, `SL1`-`SL18` | **Plan A** complete and merged (`9ed536f6`). **Plan B** (SPH fluid) unwritten. **Plan C** (the sandbox) **WRITTEN 2026-09-17** -- `plan-c-sandbox.md`, unexecuted; the v1-quarantine half stays with Plan B's blocker. ✅ **`SL1`-`SL18` SIGNED 2026-09-17** -- s.3.1 |
| Sim host and training connectors | 17th, `HS1`-`HS12` | Signed off 2026-08-19. `F-5` amended from outside by `TS11` |

### 3.1 - The `RS` and `SL` sign-off, 2026-09-17

✅ **`RS1`-`RS15` and `SL1`-`SL18` are SIGNED.** Recorded here because a signature is the most
consequential thing this document carries: everything downstream treats these rulings as ratified on
this record's word.

**What was signed, precisely -- the ENUMERATED LISTS, not the ranges:**

| | |
|---|---|
| **`RS1`-`RS15`** | Signed as the enumerated **20**: `RS1`, `RS1a`, `RS2`-`RS9`, `RS10a`, `RS10c`, `RS11`, `RS11a`, `RS12`-`RS15`. There is no bare `RS10`, and `RS10a`/`RS10c` are resident in `06-sandbox-and-v1.md`, not here. |
| ⛔ **`RS10b`** | **EXCLUDED. Recorded as superseded, NOT ratified.** It rules *delete v1* against `SL14a`'s *quarantine v1*, which is the rule in force. ⚠ **"Signed and superseded" and "excluded" are different claims**, and the first would quietly ratify a deletion instruction that must never execute. |
| **`SL1`-`SL18`** | Signed -- ~30 ids, including the lettered variants `SL2a/b`, `SL9a`-`SL9f`, `SL14a/b/c`, `SL15a/b`. |
| **`SL16`** | Signed on **both** legs **as of `fbfbbe8b` and by nothing earlier.** The `AGENTS.md` vocabulary entry it cites did not exist until then; *"AGENTS.md documents this"* was already believed true once, on the strength of a section with the right heading about the wrong subject. |
| **`RS15`** | Signed **as written. Post-processing stays excluded**, and that is a decision rather than a side effect -- see below. |

**Conditions, both discharged before the signature was recorded:**

* **`RS14` fixed** (`6776bfde`). It claimed to enumerate every amendment owed and was short by **two**
  rows, not the one `C10` found -- engine-design s.9, and the 24th spec itself.
* **`SL16`-`SL18` harvested** (`20ea8d45`, `fbfbbe8b`). All three had lived **only** in the superseded
  24th spec; signing before that would have ratified three rulings with no live text.

**Provenance -- stated exactly, because it is the whole subject of the night this was recorded.**
The user's own answer to a question naming **this signature specifically**, put to them for that
purpose and relayed by the coordinator session on 2026-09-17. **Not** an inference from a general
approval, and **not** the standing authorization that covers ordinary decisions.

> ⭐⭐ **THE RULE THIS PRODUCED, NOW STANDING FOR EVERY REALM: a user's signature, sign-off or
> ratification is recorded ONLY from the user's own words about that specific thing.** No relay of a
> general approval, no inference, however reasonable. Everything else routes through the coordinator
> as before; this one class does not.
>
> It came from a near miss. *"Plan looks good. Dispatch."* was read as ruling all 22 recommendations
> and relayed to six realms as such -- **a reasonable reading of an instruction to proceed, and not
> the same act as a signature.** The token was not flipped on it. **A signature closes a file harder
> than anything else we write**: *"this is unaddressed -- ask"* costs one question, while *"this is
> already settled"* closes the file, and a closed file is where a wrong finding never gets
> re-examined.

**`RS15` and the horizon blur -- ruled, not absorbed.** `RS15`'s out-of-scope list excludes
**post-processing**, and it stands. `SR-17a`'s horizon blur is admissible because it is an
**atmospheric term inside the background pass**, not a screen-space filter. ⭐ **The route was not
chosen to get around the exclusion; the exclusion is why that route is right** -- an atmospheric term
composes with the render, participates in the determinism story and means the same thing on every
backend, while a post-hoc image filter would have to be excluded from, or would move, goldens whose
manifest asserts **bit-identity across platforms**. A screen-space implementation remains off the
table without a further amendment.

**Machine-readable state:** `06-sandbox-and-v1.md` carries `<!-- signoff-state: SL1-SL18 SIGNED -->`,
read by `tools/tests/test_spade_sandbox_gate.py` (`PC21`). On the flip that gate **changed polarity
rather than being deleted** -- it stops asserting the `spade_sandbox` target's absence, and
`docs/dev/executables.md` takes over asserting its existence once it is built.

**Plan A delivered:** the object graph and ten registered component types; JSON serialization; the
behavior registry and its two fixed schedule slots; `kinematic_mover`; `DrawMode::velocity`;
seeded `spawn_in_sphere`/`spawn_in_cube`; and the machine-checked v1 transfer register.
`kSnapshotVersion` did not move and **no golden file changed** -- the structural addition cost the
determinism estate nothing, which is `SL3` working as designed.

---

## 4 - Standing rules adopted during execution

Forced by evidence during S1-S6; they now carry the same weight as the specs' own constraints.

1. **Bit-portability.** No libm transcendental on any path feeding registered state or a digest --
   `core/fp32_math` kernels only (IEEE-mandated ops: `+ - * / sqrt`). Enforced by a comment-aware
   source-scan canary whose pattern set covers C spellings and glm transcendental wrappers.
2. **fp32 text round-trip = charconv only.** 9 significant digits via `std::to_chars`, parsed via
   `std::from_chars`. `snprintf`/`strtof`/iostreams/yaml-cpp `as<float>()` forbidden -- locale
   immunity, because the Qt editor host calls `setlocale`.
3. **House spelling for authored rotations:** sqrt forms or literal correctly-rounded constants,
   never `glm::angleAxis` -- its `sinf` measured 0.038 ulp from a rounding midpoint, a genuine
   cross-libm coin flip.
4. **Golden discipline.** Regeneration only with root-cause provenance, cross-checked on gcc-13 CI
   before the commit finalizes; never loosened, never per-platform. A Linux-only divergence is a
   bit-portability defect, never a tolerance adjustment.
   > ⚠ **CORRECTED 2026-09-17. This rule used to end "Whole-history outcome: exactly *one*
   > regeneration event across every program." That was false, and the goldens' own changelogs
   > refute it.** Measured: **at least four** regeneration events across three manifests.
   > `render/frames/manifest.json` records **two** -- Task **R6** (*"REGENERATED … materials/
   > lighting/sky/analytic-ground landed"*) and Task **VQ-A** (*"REGENERATED, all four hashes
   > moved"*, deliberately kept in a separate `_changelog_vqa_a` key so the two stay
   > distinguishable) -- and `render/csg/manifest.json` and `render/tessellation/manifest.json`
   > each record one in their fix waves.
   >
   > **The correction matters because the false half made regeneration sound extraordinary, and
   > scarcity was never the discipline.** The discipline is the other sentence: root-cause
   > provenance plus a cross-platform check. Measured against *that* bar the record is good --
   > Task **R7** is the model, having **INVESTIGATED and NOT regenerated**: it recorded why each
   > hash was expected to move, proved mathematically why it did not, and left two
   > mutation-verified regression tests standing where a golden would have. **A rule that
   > overstates how rarely something happens teaches the next reader that doing it correctly is
   > a transgression** -- which is how a stale golden gets argued into staying stale.
   > Regeneration here is routine, governed and auditable, and that is the claim the evidence
   > actually supports.
5. **State registration is append-only.** The snapshot walk is pinned by test. `config_hash` folds
   the whole `WorldSetDesc` fail-closed, with a `sizeof` guard forcing every future `WorldDesc`
   field to classify itself.
6. **One validation.** All four world-entry paths -- `WorldBuilder::build()`, `load_world_file`,
   path-ref, desc-ref -- run the same `validate_world_desc`.
7. **Demo-scene rules.** FPS counter in every scene; deterministic spawn perturbation for large
   object counts unless exact stacking is a deliberate fault test; v1 regression scenes stay
   runnable. *(These were the S1-S4 record's local `A1`-`A3`; renamed here -- see `RECONCILIATION.md` `C7`.)*
8. **Tolerance policy.** Measured then pinned with margin; calibrate cross-platform and
   cross-device before pinning.
9. **GPU bit-portability corollary.** Parity-path kernels never call vendor transcendental
   intrinsics -- only the Slang ports of `core/fp32_math`. `NoContraction` + denormal-preserve
   pinned on every kernel (the box iGPU flushes denormals by default -- the GPU analogue of the
   libm lesson). Machine-enforced by SPIR-V scan rules **`P1`** (NoContraction) - **`P2`** (no
   `OpDot`/sum-of-products -- slangc cannot decorate them, accumulation order unspecified) -
   **`P3`** (denorm-preserve declared) - **`P4`** (no Int64 capability) - **`P5`** (no
   GLSL.std.450 exp/log/trig) + **`E1`/`E2`** (no `OpFDiv`/`sqrt` in fp32_math modules).
10. **Ordered accumulation / workgroup-size invariance by construction.** No workgroup-shared
    reduction whose order depends on local size on any parity path. `workgroup_size` is a live knob
    ({32, 64, 128}) and the sweep is discriminating. **GPU results are never a golden source;**
    parity compares live.
11. **Record-once command buffers.** The full dispatch chain is recorded once per shape; per-step
    data flows through a persistently-mapped `StepParams` buffer.
12. **Installed-tree self-containment.** The installed `spade` package must link clean for an
    out-of-tree consumer on both platforms.
13. **One invariant, one site.** *(Added by Plan A.)* A recycled slot is reset in `create()` and
    nowhere else. Two sites maintaining one invariant make each other untestable -- neither can be
    removed without the other silently covering, so no mutation reaches either one.
14. 🔴 **GPU PARITY IS DEVELOPER-MACHINE-ONLY, AND MUST BE CITED THAT WAY.** *(Declared 2026-09-17.)*
    **No CI job in this repository has ever observed a live GPU parity comparison pass or fail.**
    Two measured facts compose into it, and neither is visible from the other's side:
    `spade/tests/` is unreachable from every root build tree, so its **only** automated gate is
    `.github/workflows/spade.yml`; and that workflow's runners **have no GPU**, by its own comment
    (*"This runner has no GPU: `spade_compute` BUILDS (that is what this flag proves), and the
    gpu-labeled tests SKIP"*). `ctest --no-tests=error` fires only on **zero registered** tests, so
    a permanently-skipping battery is invisible to red/green forever.

    **What that does and does not cover -- state the layer, never "GPU coverage" bare:**

    | layer | in CI? | what it is |
    |---|---|---|
    | **Static SPIR-V / layout rules** -- `P1`-`P5`, `E1`/`E2` | ✅ **RUNS** | `test_slang_layouts.cpp`, **9 tests, none device-gated**. `spade_compute` builds and the compiled artifact is scanned. Rule 9's decorations are genuinely enforced. |
    | **CPU-side halves of the parity files** | ✅ **RUNS** | 6 ungated sites, incl. `ParityCorpus.EveryCorpusScenarioIsInTheParitySet` -- a **membership floor** over the parity corpus, so a scenario silently dropping OUT of the compared set still reds. |
    | **Live GPU parity, invariance, state mirror** | 🔴 **NEVER** | **61 sites** across five files, all `if (!vulkan_available()) GTEST_SKIP();`. |

    **So the honest sentence is: the compiled artifact's discipline is enforced and the question is
    still asked of every scenario -- what no gate has ever checked is whether the two backends
    AGREE.** Rule 10 makes that gap load-bearing: *"GPU results are never a golden source; parity
    compares live"*, and the live comparison is exactly the half that does not run. `SL18`'s
    cross-backend determinism obligation rests on it.

    **The resolution is this declaration, not deletion and not silence.** These tests are correct
    and worth keeping; a device-gated skip is the right behaviour for a machine with no device.
    ⭐ **The defect was never the skip -- it was that a 61-test battery was being counted as
    coverage by readers who had no way to learn otherwise.** Anyone adding a device-gated test runs
    it on real hardware and says so in the commit, because nothing downstream will. The same
    declaration sits at the top of `spade/tests/CMakeLists.txt`, where the hand actually is.

---

## 5 - Adjudications not to re-import

- `install(EXPORT)` **rejects** `$<INSTALL_INTERFACE:yaml-cpp::yaml-cpp>` regardless of genex
  wrapping. Private-static-dependency closure lives in `spadeConfig.cmake.in` as a config-time
  `set_property` append. Root-caused empirically with a Kitware citation (S5 T8).
- `VehicleSpawn::rotor_omega` is a **scalar** (common shaft speed) by engine design. The S5 plan's
  claim otherwise was wrong and was overridden.
- The scenario schema is engine-owned **test infrastructure** and was never user-frozen -- only the
  world schema was. It may grow with review-adjudicated cause.
- The `.digest` sidecar files are retired; all four values are carried byte-identical elsewhere.

---

## 6 - Open items

### Blocking

| Item | Why it blocks | Owner |
|---|---|---|
| **SPH fluid is the one open transfer-register row** | v1 remains the only implementation of the fluid solver. **v1 must not be quarantined while this row is open** -- doing so would retire a live capability rather than a superseded one. `SL14a`'s quarantine is therefore specified but correctly unexecuted | Plan B (unwritten) |
| **29 of the 40 `SR-nn` rulings the code cites exist in no specification** | ⚠ **A CLASS, not an oddity, and the next realm to meet one should know that.** Measured 2026-09-17 at master: **40 distinct `SR-nn` cited across `spade/`, `tools/`, `content/`, `dronesim/`, `editor/`, `runtime/`; 15 appear anywhere in `design-specs/`; 29 orphaned** -- `SR-3 9 10 11 13 14 15 17 18 21 22 23 24 25 27 28 30 31 33 34 35 37 38 39 40 41 52 57 58`. **CAUSE: the render program's rulings were harvested into `03-world-and-render.md` under a RENUMBERED series (`RS-*`), and the `SR-*` numbers the code actually cites were never carried across** -- so an `SR-nn` in a comment points at a series this spec set abandoned, and **reads as settled authority to anyone who does not go looking.** `SR-17` is now written (`03-world-and-render.md` s.15) and is the model, including its archaeology method and the two citation disagreements it turned up. ⭐ **USER RULED 2026-09-17: reconstruct the rest AS-TOUCHED, not in one pass** -- the phase that needs one writes it first. ⚠ **Superseding, deleting or "clarifying" an orphan without reconstructing it first repeats exactly what `SR-17` nearly cost**: a wholesale supersession there would have retired six unrelated clauses, including a load-bearing bit-identity seam with a golden pinned to it | as-touched, per realm |
| **No guard ties a header's version constant to its surface** | `KATHOST_ABI_MINOR` (C5) and `KAT_ABI_MINOR` (C2) are both correct today, and C5's has correctly never moved -- but by attention, not by construction. Neither header carries a machine-readable MINOR-vs-MAJOR rule, and **no check would fail if the next additive call skipped its bump.** A missing guard, not a defect. SDK holds the C2 half; `../runtime/07-status.md` s.1 carries it as a `GAP` | cross-realm |

> **WITHDRAWN 2026-09-08 -- "C5's semver has never moved" was not a defect.** For part of this
> day the row above read *"C5's semver has never moved: `KATHOST_ABI_MINOR` is still 0 although
> `B2`'s `kathost_caps_query` shipped as an additive call."* **Every fact in it was true and the
> conclusion was false.** The `/* B2 */` tag beside the call records **which amendment specified
> it, not when it entered the file**: `git log -S` returns the same single commit (`eebc4373`,
> 2026-08-11) for the constant and the call, and diffing the declared symbol set between that
> commit and today gives the identical 23 names -- nothing additive ever shipped unversioned.
> Raised by the runtime realm, withdrawn by them the same day after SDK checked the history
> behind the annotation; re-verified here independently before the withdrawal was accepted. The
> row now states what actually survives, which is the missing guard.

> **CLOSED 2026-09-08 -- C5 finally has a section.** The long-standing entry here read *"C5 has
> no section in the contracts spec: errata `R1`'s obligation was never fulfilled, so `TS11`'s
> amendment of C5 has no canonical text to point at."* The runtime realm wrote
> `design-specs/runtime/04-host-c5.md`, which discharges `R1` and gives `TA5`, `MV1` and `TS11`
> a target. The three-layer split -- tracked header / contract / Spade implementation -- is
> recorded in `../integration/spade-c5-adapter.md`'s opening block. This row is replaced above by the finding that
> came out of writing it.

### Carried forward from the S1-S4 record

| Item | Status |
|---|---|
| A body resting on the ground reads ~0 specific force (contact impulses bypass `force_acc`); in-flight readings are correct. Any ESKF-initializes-on-ground assumption on the Kat side must know this | **still open** -- documented in `imu.hpp`; owner was "S7 contact-model question", and S7a did not touch it |
| Reaction torque Q uncorrected by inflow/ground factors while thrust is corrected (spec-literal); rotor Q is not a power budget | **still open** -- documented in `rotor.hpp`; revisit with the aero roadmap (S8/`P7`) |
| Hover scene's orbit camera is wall-clock-driven, so the composited image is not pixel-reproducible across launches (physics stays tick-exact) | **accepted**, disclosed as a tools exemption |
| `config`/`dt_ns`/`substeps` outside the snapshot blob | CLOSED S5 T3 |
| No `reseed()` surface | CLOSED S5 T4 |
| Remaining libm transcendentals | CLOSED S5 T1 (the single golden regeneration event) |
| `VehicleRef` not reconstructible after restore | CLOSED S5 T7 |
| `ModelType::proxy_radius` declared but not consumed | CLOSED S6 T2 |
| Microbenchmark spread makes `BM_IntegrateOrientation` unusable as a regression basis | CLOSED S6 T10 (bench spread policy) |
| `ecs/` had zero production consumers since S1 | CLOSED S5 T9 -- deleted |

### Deferred by design

- **`S7b`**: Vulkan render backend, ImGui debug HUD, camera sensors, `FramePool` on the GPU path.
- **`P7`**: promoting effects to aero (the SPH-to-aero promotion criteria).
- Full heterogeneous per-world geometry beyond the two-world regression case S6 added.

---

## 7 - Traceability

**S1-S4** (`spade/s1-s4`, base `3ee7894`, merge `6772f0c`):

| Tasks | Delivered | Commits |
|---|---|---|
| T1-T4 | Scaffold/presets/core - math ops + scripts - v1 deletions + pins - CI + bench (S1 gate) | `e85652c` `45ebea9` `c831220` `76aa0d4` |
| T5-T7 | ECS - state layout/arenas - snapshot (S2 gate) | `42795df` `26acab1` `4d53fce` |
| T8-T10 | RNG + medium v0 - integrator - SDF worlds | `5bb9b68` `eeab0bc` `dcffd89` |
| T11-T13 | Contacts - sorted grid - schedule/Simulation/corpus | `d4b9887` `f6961fa` `a67d04c..c088179` |
| T14, T14b | Viewer + demo scenes (S3 gate) - FPS/jitter/v1-regression amendments | `76b5b00`+`8a2a521` - `c86c443` |
| T15-T17 | Force elements + drag - Dryden - rotor | `1ae2a79` `686e339` `9194095` |
| T18-T20 | ModelType + Quadrotor - IMU + rings - flight scenes + M1B bar | `f03c44b` `ec5289f` `0e2e29d` |
| T21 | Bench sweep - docs - mechanical batch - review fix wave - parity fix | `2667077` `3b34617` `1525114` `2533082` `5cd3a63..9de1f75` |

**Later phases:** S5 `6bc6b5c` - S6 `153275f` - S7a `9778260` (CI `e6e3082`) - SL Plan A `9ed536f6`.

**Durable records outside this file:** git history; `spade/CHANGELOG.md`; `spade/README.md`;
`spade/engine/objects/README.md`; `spade/docs/v1-transfer-register.md` (machine-checked); the
golden digest provenance headers.

---

## 7b - The three cross-host SDK tests: DECLARED ENVIRONMENT -> MEASURED GREEN (2026-09-18)

Three `sdk/tests/cpp` cases were being carried in the phase record as *"declared environment reds --
need `-DKAT_BUILD_SPADE_HOST=ON`."* **A declaration is a real outcome, but it is not a measurement.**
This converts them.

**RESULT -- BY NAME, never as a delta in a total:**

| test | result |
|---|---|
| `SensorWireRealHosts.TheProbesWireMatchesBothRealHostsOnTheCapacityLadder` | ✅ **Passed** 0.31 s |
| `RolloutHosts.TheSameRolloutRunsOnBothHostsWithSharedWireSemantics` | ✅ **Passed** 0.49 s |
| `RolloutHosts.TheDetectedEnvelopeDiffersBetweenTheHostsAndTheTraceSaysSo` | ✅ **Passed** 0.23 s |

**`3/3, CTEST_EXIT=0`.**

⚠⚠ **REPORTED BY NAME BECAUSE A COUNT CANNOT ANSWER THIS QUESTION.** `--no-tests=error` fires only on
**zero** registered tests, so *"3 fixed"* and *"3 never ran"* are indistinguishable in a total --
this realm's own 61-GPU-test finding, arriving in another realm's suite. ⭐ **These three are
registered unconditionally and `ADD_FAILURE()` rather than skipping** when `KATSDK_HOST_SPADE_LIB_PATH`
is undefined, which is what makes them answerable at all: *a case that skips when its real subject is
absent has quietly substituted a different subject.*

**THE PRECONDITION, CHECKED BEFORE SPENDING THE BUILD** (the cheap exit, pre-registered):

```
KATSDK_HOST_SPADE_LIB_PATH in build.ninja      23 hits
kat_host_spade target present                  689 references
NEGATIVE CONTROL: a nonsense macro name        0 hits   <- so the grep discriminates
```

**`SUBJECT:`** Kat **lean + SDK** tree (`build-spadehost`), **not** the `spade/` standalone tree.
**`RUN:`** `-DKAT_BUILD_EDITOR=OFF -DKAT_BUILD_TRAIN=OFF -DKAT_BUILD_SDK=ON -DKAT_BUILD_SPADE_HOST=ON
-DKAT_SPADE_PREFIX=<repo>/spade/install-host`, `CMAKE_BUILD_PARALLEL_LEVEL=1`.
`CONFIGURE_EXIT=0` (164.9 s) · `BUILD_EXIT=0` (1466 s, target `katsdk_core_tests`, which pulls
`kat_host_spade` through `add_dependencies` -- **freshness is per-consumer, so the consumer target is
what was built**) · `CTEST_EXIT=0`.

⚠ **DEVIATION, STATED:** the MSVC environment was imported inline (`vcvars64.bat`) rather than going
through `editor/scripts/ninja-build.ps1`, **because that script hardcodes its `-D` flags and cannot
express `KAT_BUILD_SPADE_HOST=ON`.** A bare `cmake` without it fails with *"Tell CMake where to find
the compiler"* -- ⭐ **a toolchain failure, not a source defect**, and the same class as this session's
`C1083: cannot open 'cstdint'`.

### ⚠ THE TASK HAD A THREE-STEP PREREQUISITE CHAIN NOBODY HAD COSTED

It was offered as *"take the box and run them."* It is:

```
1. spade/install-host ABSENT -> find_package(spade CONFIG REQUIRED) fails configure outright
   scripts/spade-prefix.ps1 does a SEPARATE SCRATCH BUILD (spade/build-host)   1815 s
2. configure the lean+SDK tree                                                  165 s
3. build katsdk_core_tests + kat_host_spade                                    1466 s
4. run the three                                                                1.2 s
```

⭐ **Fifty-six minutes of prerequisite for one second of measurement**, and the prerequisite was
invisible in the request. ⚠ **Pricing it before spending it is what made the ask answerable** -- the
alternative was discovering step 1 after committing to the slot. ⭐ And `spade-prefix.ps1` prints its
own honest banner when `-SkipCTest` is used: *"TESTS SKIPPED: spade/install-host was produced by a
tree nobody tested"* -- **an instrument that refuses to let its own shortcut go unrecorded.**

---

## 7a - Platform claims in engine comments: swept 2026-09-18, and the result is CLEAN

Prompted by another realm's finding: a comment asserting that a UI framework painted controls
natively *"regardless of QSS"* had been **true when written and became false on a version bump no
document mentioned**, and had been read as a live constraint for months. It was the stated reason
for a whole design decision. **The same shape is available to any engine comment that asserts what
a compiler, a CPU or a driver does.**

**121 comment lines in `spade/engine/` assert platform, compiler or driver behaviour.** Sampled
across the highest-density files -- `compute/vulkan/context.cpp`, `compute/step_params.hpp`,
`core/fp32_math.{hpp,cpp}`, `render/scene.hpp`.

**THE SWEEP FOUND A THIRD CATEGORY, AND IT IS THE ONE THAT MATTERS:**

| | what it looks like | can it rot silently? |
|---|---|---|
| **ENFORCED** | the code **checks the claim at runtime, or the build does** | ⛔ **No.** It cannot become false without something going red |
| **RECORD** | *"measured on device D / toolchain T"* -- the subject is named | ⚠ No, because it never claimed to be general |
| **PIN** | present tense, no device, no date, no check | 🔴 **Yes. This is the P4.7 shape** |

**✅ EVERY SAMPLED CLAIM WAS ENFORCED OR A RECORD. No PINs found.** Examples, because a negative
result is only worth anything if it can be checked:

* `context.cpp:66` -- *"does this physical device preserve fp32 denormals in shaders?"* is not a
  comment's claim at all: **`create()` REFUSES a device that does not.** ⭐ **ENFORCED** -- the
  strongest form, and the model for the rest.
* `context.cpp:52` -- *"the box this program develops against (an Intel Iris Plus) exposes exactly
  one queue family, so it is what every `GpuContext.*` test below actually exercises."* **RECORD**,
  and note what it does: it names the device **and** states the consequence for COVERAGE rather than
  for correctness. The code handles both branches regardless.
* `scene.hpp:286` -- *"MSVC accepts the attribute there silently; gcc-13 rejects it under
  `-Werror=attributes`"* -- **RECORD of an event from this phase**, and **ENFORCED**: the gcc leg
  fails again if anyone undoes it.
* `fp32_math.hpp:22` -- *"the MSVC CRT and glibc both implement …"* -- **RECORD** of a measured
  divergence that really broke a scenario, and the whole file is the **ENFORCEMENT**.

⭐⭐⭐ **WHY THIS TREE IS CLEAN, AND IT IS STRUCTURAL RATHER THAN VIRTUOUS: A PLATFORM CLAIM IS SAFE
EXACTLY WHEN SOMETHING RE-EXECUTES IT.** Every float and toolchain claim above is re-executed by two
builds and a **byte-pinned golden** on every gate run. ⚠ The other realm's claim was re-executed by
**nothing**, because **no test could run it** -- offscreen the call returns `Unknown` and on the real
platform `Dark`, *same call, same binary, two answers.* **The comment was not worse; the enforcement
was absent, and no diligence in the writing could have supplied it.**

⚠⚠ **SO THE AT-RISK POPULATION IS NOT "COMMENTS THAT SOUND CONFIDENT" -- IT IS CLAIMS ABOUT
BEHAVIOUR NO GATE EXERCISES.** In this tree that is the GPU surface, whose 61 tests skip everywhere
automated (section 6). **The sampled GPU claims survive only because they happen to be `ENFORCED` at
runtime or scoped to a named device** -- ⭐ **they are safe by their own construction, not because
the battery is watching them, and a future GPU comment written as a PIN would have nothing to catch
it.**

**FLOOR, stated because a clean result invites over-reading: this was a SAMPLE, not a classification
of all 121.** It covered the highest-density files and every claim load-bearing for a golden. **A
`PIN` in a file I did not open would not have been seen**, and the sweep's own instrument was a grep
for a keyword list, which cannot find a platform claim phrased without one of those words.

---

## 8 - The one parity break worth remembering

Four golden scenarios were re-baked on Linux; one failed at tick 0, before any physics ran, and
**three passed by luck**. It is the single most instructive failure in this program's history and
it is why standing rules 1 and 9 exist.

**The full account -- mechanism, why the other three passed, and what was deliberately not done --
is `08-lessons.md` section 1.1.** It is not repeated here; it was told twice and this is the copy
that had no reason to exist.
