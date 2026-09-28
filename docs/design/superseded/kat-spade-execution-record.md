# Spade execution record — spec vs. built

**What this is:** the running scorecard of the Spade v2 engine build against its spec of record
(`kat-spade-engine-design.html`, the ninth spec, approved v1.0 2026-08-08 + Addendum A signed
2026-08-10). One row per spec obligation; each row names its evidence. Maintained forward — updated
at every phase close-out. Companion narrative for S1–S4 lives in
`kat-spade-s1-s4-implementation-record.html`; the changelog carries the dated close-out entries.

**State as of 2026-08-13:** phases **S1–S5 COMPLETE and MERGED** (S1–S4: PR #5 → master `6772f0c`,
2026-08-11 · S5: PR #13 → master `6bc6b5c`, 2026-08-13). Spade at **v0.2.0**. Suite: **437/437**
registered tests green on both MSVC presets and gcc-13 Linux CI (436 pass + 1 designed env-gated
skip). **Charter M1B bar MET on the CPU path** (S4, per Addendum A11). Next phase: **S6**.

**State as of 2026-08-17:** phase **S6 (Vulkan compute backend + CPU↔GPU parity) COMPLETE and
MERGED — checkpoint №2 passed, PR #15 → master `153275f`** (exit gate satisfied on local
evidence; the hosted-CI record rides the first post-quota-reset run on master; draft PR #14
closed as superseded). All 8 substep passes run device-resident on Vulkan; **full-corpus
CPU↔GPU parity green** (worst rel 8.27e-5; bounce and contact-pair rows bit-identical; every
integer lane bit-exact); fp32_math Slang ports **bit-exact on device over 99,734,618 args, 0
mismatches**; A7 knob invariance **discriminating** (live `workgroup_size`, mutation-killed sweep);
bench recorded (64-world batched **1.014× realtime on the box iGPU, real_time basis** — the
NOT-VALIDATABLE-ON-THIS-HARDWARE posture governs the spec's dev-GPU target; TR12 cited in
`baselines.json:_meta`). Suite: **521/521 both MSVC presets** (59 device tests live on the Iris) ·
**SPADE_VULKAN=OFF 442→445/445** · gcc-13/Linux **521/521 end-to-end incl. install+consumer**
(local Docker replica — GitHub Actions monthly quota exhausted; CI record rides the first
post-reset run, draft PR #14 open). Heterogeneous-worlds coordination item (training spec §4)
ANSWERED at checkpoint №1: engine admits per-world geometry; T6b closed the last per-batch config
record + landed the bidirectional regression lock; full heterogeneous product work staged S7+/TR5.

---

## 1 · Decision record D1–D12 — implementation status

| # | Decision | Status | Where / evidence |
|---|---|---|---|
| D1 | Symplectic Euler + quaternion exp-map, substeps, integrator = pure fn | ✅ BUILT (S3) | `physics/integrator.*`, `core/math_ops.hpp`; energy + ballistic validation suites |
| D2 | fp32 both paths, pinned reduction order, local-frame worlds | ✅ BUILT — and STRENGTHENED | Whole engine fp32; **bit-portability went beyond spec**: all libm transcendentals retired to in-engine `core/fp32_math` kernels (log32/sin32/cos32/exp32, ≤1 ulp exhaustively verified, totality-guarded at every magnitude) after a real 1-ulp MSVC-vs-glibc parity break. Cross-platform bit-identity is a PROVEN contract (identical digests MSVC/Windows ↔ gcc-13/Linux, CI-enforced every commit) |
| D3 | Analytic SDF static worlds + sorted-grid dynamic-dynamic; mesh colliders later | ✅ BUILT (S3) | `world/sdf.*` (primitives + CSG + heightfield, flat postfix program), `physics/` grid; triangle meshes remain a later add-on as specified |
| D4 | Model-type layer, Quadrotor first | ✅ BUILT (S4) — gap CLOSED (S6) | `vehicles/model_type.*`, `vehicles/quadrotor.*`. The carried per-body `proxy_radius` gap closed by S6 T2 (D-S6-2): `BodyState.proxy_radius` (sentinel-0), `effective_proxy_radius()` the ONE canonical predicate, both collision passes + the GPU kernels consume it; quad_hover golden regenerated with full provenance (gcc-13 digest reproduction) |
| D5 | Rotor = curves + RPM lag + momentum-theory inflow + SDF ground effect; BEMT = P7 roadmap | ✅ BUILT (S4) | `vehicles/rotor.*`; climb/descent vs momentum-theory reference curves in the validation suite |
| D6 | Medium interface; Dryden seeded turbulence | ✅ BUILT (S4) | `world/medium.*`; Dryden spectral sanity + assembled-recursion bias law verified at operating θ (S5 T1) |
| D7 | WorldBuilder + versioned YAML world file; glTF later, render-only | ✅ BUILT (S5) | `world/builder.*`, `world/world_file.*`. **Schema v1 USER-FROZEN 2026-08-12** (strict unknown-keys, `schema_mismatch` version gate, uniform op-node shape — all user-ratified; evolution = version bumps). Seven committed worlds, byte-pinned by the writer-match corpus test |
| D8 | Native world-index batching everywhere; N=1 degenerate; CPU twin serial | ✅ BUILT — BOTH HALVES (S6 closes the GPU half) | CPU: `(world_id, cell)` grid keys, per-world capacities, world-partitioned SoA arenas. GPU (S6): batched sorted-grid dynamic collision (bitonic over `(world_id,cell)`, both batching paths, CPU-tie-break-exact), per-world config buffers throughout (`contact_params`/`grid_params`/`dryden_params` by world — no per-batch config record), GPU batching-invariance bit-identical (solo vs in-set), heterogeneous per-world geometry admitted + regression-locked bidirectionally (T6b) |
| D9 | Slang single-source layouts → generated C++ headers + binding registry | ✅ BUILT (S6, check-generation posture per D-S6-1) | `cmake/SpadeSlang.cmake` + Python generator: slangc-reflection → `layout_check.gen.hpp` (per-field offset/size static_asserts, 236+) + `bindings.gen.hpp`; 28 SPIR-V modules (9 kernels × 3 workgroup sizes + probe) machine-scanned by rules P1–P5 + E1/E2; full source-inversion stays S7+ (same machinery, flip later) |
| D10 | Strangler: v1 GL + sandbox untouched until S7 | ✅ HELD | v1 (`spade/src`, `spade/include`, `spade/assets`) untouched through both programs; viewer/demo scenes run on it today |
| D11 | GTest/CTest from first commit; determinism + parity spine; own CI; bench in-repo | ✅ BUILT — parity spine COMPLETE (S6) | 521 tests; **CPU↔GPU parity suite live over the full committed corpus** (bands measured-then-pinned with device provenance, abs-OR-rel with `cpu != 0` rel semantics; three-grade determinism honored: CPU↔CPU bit-identical · GPU↔GPU bit-identical per device+driver · CPU↔GPU banded); invariance battery (A7 knobs discriminating, batching, cross-backend restore); bench = GPU sweep w/ per-pass Vulkan timestamps, spread-policy ticket DISCHARGED, baselines re-seeded. CI record pending quota reset (local gcc-13 Docker gate green end-to-end at tip) |
| D12 | MSVC + Ninja + presets, C++23, std::expected spine | ✅ BUILT (S1) | `CMakePresets.json`; `Result<T>`/`spade::Error` everywhere; no exceptions cross the boundary |

## 2 · Phasing S1–S8 — exit proofs

| Stage | Spec exit proof | Status / evidence |
|---|---|---|
| S1 | CI green on math unit suite; bench baseline emitted | ✅ (2026-08-10/11 program; `spade::` install/export landed — hardened in S5 by the out-of-tree consumer smoke, which caught a real EXPORT_NAME defect) |
| S2 | Lifecycle + snapshot round-trip green | ✅ (registry-walk snapshot blobs; restore = byte-faithful; the S1-era `ecs/` scaffold was superseded by `state/` arenas and **deleted in S5 per decision D-S5-1** — physics never ran on it) |
| S3 | Determinism replay green; ballistic/energy green | ✅ (golden corpus born here; digest = FNV-1a fold over the registration walk) |
| S4 | Hover-trim + climb/descent green; **charter M1B bar (headless CPU)** | ✅ **M1B MET** — `test_m1b_bar.cpp` charter-bullet asserts + Addendum A3 conformance cases (reset-preserves-roster, reseed determinism); Spade → 0.2.0 (A11). Rate targets HIT on CPU: ~296k substeps/s single-world (30× the 10 kHz floor); 64-world batched ~2.8× realtime (CPU-twin proxy for the S6 GPU claim) |
| S5 | **Corpus is data; C5 `world_ref` loadable** | ✅ + substitution absorbed (Addendum note (iii)): five `.scenario.yaml` + seven worlds; `.digest` files retired with all four values carried byte-identical; builder≡data equivalence is a committed bridging test with an independent hard-coded second source; `world_ref` (path\|handle) installed and consumer-proven; `vehicle_ref_at`, `reseed`, replay-config restore gate all live. Full record: `spade/README.md` §"S5 exit record" + PR #13 body |
| S6 | Parity bands green on the box; batching-invariance green; bench sweep recorded | ✅ **EXIT GATE SATISFIED 2026-08-17** (branch `spade/s6` @ `bc1291e`; awaiting checkpoint №2 → PR) — parity bands green over the FULL corpus on the box (worst rel 8.27e-5; bounce/contact_pair bit-identical; integer lanes bit-exact) · batching + knob invariance green (workgroup sweep DISCRIMINATING: live knob, mutation-killed) · bench sweep recorded w/ per-pass timestamps (64-world 1.014× realtime real_time-basis on the Iris; iGPU posture + TR12 in `_meta`) · fp32_math bit-exact on device (99.7M args/0) · gcc-13/Linux end-to-end green incl. install+consumer (local Docker; hosted-runner record post-quota-reset) · 13 tasks (T1–T11 + T6b + T9b inserted by checkpoint/finding rulings), every task review-gated, whole-plan opus review clean after 1 fix wave, rulings audit zero violations |
| S7 | GL sandbox retired; headless/rendered equivalence green | ⏳ (FramePool/TA1 design pinned in spec §9; camera-sensor seat opens here per A8) |
| S8 | Effects demos under Vulkan; promotion rule documented | ⏳ |

## 3 · Standing rules adopted during execution (bind all future phases)

These were forced by evidence during S1–S5 and now carry the same weight as the spec's own
constraints (canonical texts: the plan files' Global Constraints + `kat-spec-changelog.md`
2026-08-11/12 entries):

1. **Bit-portability rule.** No libm transcendental on any path feeding registered state or a
   digest — `core/fp32_math` kernels only (IEEE-mandated ops: `+ − × ÷ sqrt`). Enforced by a
   comment-aware source-scan canary (`test_m1b_bar.cpp`) whose pattern set covers C spellings and
   glm transcendental wrappers.
2. **fp32 text round-trip = charconv only.** 9 significant digits emitted via `std::to_chars`;
   parsed via `std::from_chars`. `snprintf`/`strtof`/iostreams/yaml-cpp `as<float>()` forbidden
   (locale immunity — the Qt editor host will setlocale). Amended over the original %.9g/strtof
   wording at the S5 T5 review.
3. **House spelling for authored rotations:** sqrt forms (e.g. `sqrt(0.5f)`) or literal
   correctly-rounded constants — never `glm::angleAxis` (its `sinf` measured 0.038 ulp from a
   rounding midpoint = a genuine cross-libm coin flip).
4. **Golden discipline.** Regeneration only with root-cause provenance, cross-checked on gcc-13 CI
   before the commit finalizes; never loosened, never per-platform; a Linux-only divergence is a
   bit-portability defect, never a tolerance adjustment. Whole-history outcome so far: exactly ONE
   regeneration event (S5 T3's replay-config suffix append) across both programs.
5. **State registration is append-only**; the snapshot walk is pinned by test (`replay_config`
   last, indices 16/17). `config_hash` folds the whole `WorldSetDesc` (fail-closed), with a
   field-set guard (`sizeof` static_assert) forcing every future `WorldDesc` field addition to
   classify itself.
6. **One validation.** All four world-entry paths — `WorldBuilder::build()`, `load_world_file`,
   path-ref, desc-ref — run the same `validate_world_desc` (user-ratified fail-closed posture,
   2026-08-13).
7. **Demo-scene standing rules (S1–S4 user amendments):** FPS counter in every scene;
   deterministic spawn perturbation for large object counts (unless exact stacking is a deliberate
   physics-fault test); v1 regression scenes stay runnable.
8. **Tolerance policy for S6 parity (spec §9, reaffirmed):** measured then pinned with margin —
   calibrate cross-platform/cross-device before pinning.
9. **GPU bit-portability corollary (S6).** Parity-path kernels never call GPU-vendor transcendental
   intrinsics — only the Slang ports of `core/fp32_math` (bit-exact on device, 99.7M-arg proven);
   `-fp-mode precise` (NoContraction) + `-denorm-mode-fp32 preserve` pinned on every kernel (the
   Iris flushes denormals by default — the GPU analog of the libm lesson); machine-enforced by
   SPIR-V scan rules **P1** (NoContraction) · **P2** (no OpDot/sum-of-products — slangc cannot
   decorate them, accumulation order unspecified) · **P3** (denorm-preserve declared) · **P4** (no
   Int64 capability — the device lacks `shaderInt64`; 64-bit state rides exact uint32-pair
   emulation) · **P5** (no GLSL.std.450 exp/log/trig ext-inst) + **E1/E2** (no OpFDiv/sqrt in
   fp32_math modules), asserted per variant on all compiled modules with a
   coverage-follows-the-build count guard.
10. **Ordered accumulation / workgroup-size invariance by construction.** No workgroup-shared
   reduction whose order depends on local size on any parity path — per-item writes or fixed-order
   serial folds only (per-world serial where the CPU is serial per world: Dryden filter,
   Gauss-Seidel contact sweep). `workgroup_size` is a LIVE knob ({32,64,128}, per-size compiled
   variants provably differing in exactly one SPIR-V word) and the A7 sweep is discriminating
   (mutation-killed). GPU results are never a golden source; parity compares live.
11. **Record-once command buffers.** The full dispatch chain (including the bitonic sort stages and
   the per-pass timestamp queries) is recorded once per shape; per-step data flows through the
   persistently-mapped StepParams buffer — never re-record, never push-constant-per-step.
12. **Installed-tree self-containment.** The installed `spade` package must link clean for an
   out-of-tree ON-path consumer on both platforms (volk's TU is absorbed into `spade_compute`;
   proven by the exit gate's install+consumer smoke on gcc-13/Linux and MSVC/Windows).

## 4 · Adjudications the S6 planner must not re-import

- `install(EXPORT)` **rejects** `$<INSTALL_INTERFACE:yaml-cpp::yaml-cpp>` regardless of genex
  wrapping — private-static-dependency closure lives in `spadeConfig.cmake.in` (config-time
  `set_property` append). Root-caused empirically + Kitware citation (S5 T8).
- `VehicleSpawn::rotor_omega` is a **scalar** (common shaft speed) by engine design — the S5
  plan's `[4]` was wrong and was overridden.
- The scenario schema (v1, `scenario_version`) is engine-owned test infrastructure and was **not**
  user-frozen (only the world schema was); it grew `drag_elements` and vehicle-spawn `vel`/`omega_body`
  during T7 with review-adjudicated cause. It is exactly the "world file + inputs + N steps" format
  spec §9 names as the parity-corpus substrate — **S6's parity harness should consume it as-is.**
- The plan's "goldens move exactly twice" became **once** — S5 T1 proved its kernel swaps bit-exact
  via perturbation probes instead of regenerating.

## 5 · Open inventory feeding the S6 plan — **ALL ROWS DISCHARGED (S6, 2026-08-17)**

Every row below was closed inside S6: per-body proxy (T2/D-S6-2) · bench spread policy (T10) ·
parser-drift guard, op-node tightening, T0-label doc, components declaration, glm closure,
guard-message polish, loader-branch tests, physics.yaml comment (all T10b). Post-S6 tickets opened
at close-out live in `kat-open-agendas.md` §6 (Path-A/maintenance4 revisit at a fourth workgroup
size · bindings 10/12 drop at the next descriptor renumber · no_op scan-profile revert pointer ·
SDF primitive parity coverage rides scenario growth · hosted-runner CI record post-quota-reset).
Original table kept for the record:

### 5a · (historical) Open inventory as it fed the S6 plan

From `kat-open-agendas.md` §6 (S5 close-out carry block + survivors):

| Item | Disposition for S6 planning |
|---|---|
| Per-body contact proxy not consumed by collision | Real engine work — candidate S6 task (GPU kernels should not bake in the wrong lookup) |
| Bench spread policy (Windows timer tick vs sub-µs benchmarks) | GATED on S6 perf-gate tooling — discharge inside S6's bench task |
| Parser-duplication drift guard (scenario vs world file) | Small; fold into S6 hygiene or first-touch |
| Op-node `transform` validation tightening | Small, no version bump; fold into first world-file touch |
| `T0` ctest label has no consumer | Wiring decision (kat-side spot-runs); S6 hygiene |
| `check_required_components(spade)` declares no components | Package polish; S6 hygiene |
| glm closure half of the package config | One-line `set_property`; S6 hygiene |
| `sizeof(WorldDesc)` guard message polish | Cosmetic; first-touch |
| Untested loader branches (explicit DrydenParams, componentwise drag, scenario-side duplicate-key) | Natural fit: S6 parity-corpus growth exercises them |
| kat `configs/physics.yaml` drag-sign comment | Doc one-liner, kat-side (outside spade) |

Also standing from Addendum A: **A7** — every `backend` config knob owes a c088179-style
invariance test (directly shapes S6's config surface); **A2/A5** — FramePool deferral + sensor-poll
discharge notes (S7 boundary); **A8** — rendered camera is S7; any M1d camera need is synthetic
feature buffers only.

---

*Update discipline: append-only per phase; each phase close-out edits §1/§2 statuses, adds its
evidence, and moves discharged §5 rows into the changelog entry that closed them.*
