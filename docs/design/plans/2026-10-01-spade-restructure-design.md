# Spade restructure — design

**Status:** SIGNED by the user 2026-10-02 (approved section by section 2026-10-01). Executed by `2026-10-01-spade-restructure-plan.md`; open user decisions are in `../backlog.md`.
**Author:** Spade lead session. **Scope:** identity, engine model, realm split, documentation library, KAT separation. Specs only — no engine code changes except the three defect fixes in section 5.

## Decisions taken (user, 2026-10-01)

| Topic | Decision |
|---|---|
| Realms | Five: **Core** (new), Physics, Rendering, Interface, Test/Docs, coordinated by the Spade lead session |
| Determinism | Fixed-step, seeding and snapshot/replay are universal; every module **declares a grade** |
| Physics composition | **Fields** (provided by regions) + **responders** (carried by objects); fidelity chosen on both sides |
| Cameras | **Technique × channel**; one camera serves as a viewport or as a sensor |
| Extensibility | **Open module system**; Spade's built-ins use the same API as developer modules |
| Drone-specific code | General parts become catalog modules; the quadrotor becomes a **template** |
| Interface realm | Grows the sandbox into **Spade's own editor** |
| Spec library | Tracked, `docs/design/<realm>/`; existing Spade rule IDs kept and frozen |
| Agent guidance | Short and broad (`AGENTS.md`, `CLAUDE.md`); clean review over enforced standards |

## 1. Identity and laws

Spade is a general-purpose, deterministic simulation and rendering engine. Developers compose worlds from regions, objects and modules (physics models, sensors, render techniques), then step them headless, batch many at once, or run and inspect them in Spade's editor. Spade carries no domain concept. Consumers build their domain from modules and templates: KAT for quadrotor training, a vehicle-dynamics tool for F1, and so on.

**Laws** (universal; Core enforces them):

- `L1` Fixed step and seeded. No clock is read in the step path, and `dt` is immutable after a simulation is created.
- `L2` Snapshot, restore and replay are first-class. Restoring under a different configuration is refused.
- `L3` Every module declares a grade: **reference**, **banded** or **best-effort**. A world's grade is that of its weakest module. A consumer may require a minimum grade; a world that cannot meet it is refused, never silently downgraded.
- `L4` What each grade requires. Reference needs a CPU implementation and a golden result. Banded needs a measured band against the reference, with a record of how it was measured. The GPU is never a golden source.
- `L5` Stepping never depends on rendering.
- `L6` No silent fallback. An unavailable backend, module or grade is either refused or announced.
- `L7` No domain in the core. A proposed addition must be useful to a caller with no interest in that domain.
- `L8` Batching many worlds is native.

**Numerics (provisional):** fp32 is the default for reference grade. A module may declare fp64 as part of its grade.

**Premises that change:**

| Was | Becomes |
|---|---|
| Every authoritative pass has a CPU twin | Only reference-grade modules need one |
| State arrays are frozen | Modules register their own state |
| A fixed ten-pass schedule | The scheduler composes module passes into phases |
| "Thin adapter above" (KAT's C5/dronesim) | Moves out of the charter into `consumers.md` |
| Effects layer with an aero roadmap | Fidelity tiers |

**KAT** consumes Spade as an installed package. Spade cites no KAT ruling as authority, its tests never read a KAT checkout, and it owns its own content.

## 2. Engine model

```
Simulation ─ fixed dt, scheduler, module set, backend
 └─ World × N (batched; one module set shared, content/params/seeds differ)
     ├─ Regions ── volumes that bind FIELDS
     ├─ Objects ── transform + components, some of which are RESPONDERS
     └─ Cameras ── objects with a technique and channels
```

- **Module** — the unit of extension. It declares the state it registers (snapshotted automatically), the fields it provides or reads, the components it defines, and the passes it contributes (each with a phase and its reads and writes). It provides a CPU implementation, optionally a GPU kernel, and a grade for each backend.
- **Field** — a named, typed quantity over space and time: gravity, medium density, flow velocity or pressure, temperature. A provider supplies it at some fidelity: constant, analytic or procedural (Dryden becomes one of these), or solved on a grid (SPH lands here as a field provider).
- **Region** — a volume (the whole world, a box, any SDF shape) with a priority. It binds field providers, and an explicit rule resolves overlaps. Regions are new; nothing like them exists today.
- **Object** — a transform plus components. A component refers to module-owned state and holds none itself. **Responders** sample fields where the object is and produce forces: rigid body, contact shape, aero at a tier, buoyancy, propulsor, and sensors (IMU, GNSS, camera).
- **Fidelity tiers** — interchangeable modules behind one responder interface. Aero, for example: T0 point drag, T1 coefficient model, T2 blade-element or panel method, T3 two-way coupled to a solved flow field.
- **Scheduler** — fixed phases per substep: Fields → Forces → Constraints/Contacts → Integrate → Sensors → Publish. Modules are placed in phases by what they read and write. Order within a phase is deterministic and part of the configuration hash. The GPU dispatch chain is derived from the same schedule; there is no hand-maintained parallel table.
- **Publish** — produces a consistent frame state: poses, plus any fields that cameras request. Rendering reads only this.
- **Camera** — a technique plus channels. A channel is colour, depth, IDs, or any registered field, so a pressure map is just a field sampled by a camera. The same camera is a viewport or a graded, tick-stamped sensor.
- **Template** — a ready-made assembly of objects, components and parameters (quadrotor, car), built on the public API and never linked into the core.
- **Grade check** — at `create()`, the world's grade is computed for the chosen backend and refused if it is below what the consumer requires.

**Inherited obligations.** Changes to the object graph queue to step boundaries once objects take part in stepping. A module with no GPU kernel and no declared fallback cannot be placed in a GPU chain: that is a refusal, never a skip.

## 3. Realms

| Realm | Owns | Code today |
|---|---|---|
| **Core** | Module API and registry; scheduler; state, snapshot, replay, config hash; world, region and object model; field registry and sampling interface; grades; backend seam and GPU-chain derivation; `Simulation`/`WorldSet` API; world-file schema; math, RNG, `Result` | `core/` `state/` `sim/` `objects/` `world/` (description side) `compute/` (seam, context, mirror, recorder) |
| **Physics** | Field providers (gravity, media, turbulence, flow solvers incl. SPH); responders (integration, contact, drag and aero tiers, propulsor, buoyancy); physical sensors (IMU, GNSS); their kernels and parity bands | `physics/` `vehicles/` `sensors/` `world/medium` and physics kernels; v1 SPH as the reference until it is replaced |
| **Rendering** | Techniques (raster, ray-trace, ray-march); channels (colour, depth, IDs, field visualisation); camera component and camera sensor; scene representation; CPU, GL and Vulkan backends; render goldens and bands | `render/` `render_gl/` `render/vulkan/` and raster kernels |
| **Interface** | The editor (the sandbox grows into it); viewer; templates and examples; packaging, install, export and the consumer project; CLI and demos; user guides | `sandbox/` `engine/tools/` `examples/` (v2) `tests/consumer/` `cmake/spadeConfig*` and install rules |
| **Test/Docs** | How things are verified and documented: harness, golden-corpus governance, parity harness, SPIR-V scanner, bench; build and gate tooling; the docs library's structure | `tests/` (each realm writes its own tests) `engine/testing/` `scripts/` build parts of `cmake/` and the structure of `docs/` |
| **Lead** | Top-level charter, engine model, realm map, register and index; cross-realm sequencing; build scheduling | — |

**How realms meet.**
- Physics and Rendering reach Core only through the module API. A change to the scheduler, state or snapshot is a Core decision.
- Physics and Rendering meet only through fields and the published frame state.
- Interface uses only the installed public API. When the editor needs more, that is a finding for Core.
- Test/Docs sets verification rules and gates; each realm owns its own tests and specs.
- Changes to the laws need the user's signature.

## 4. Documentation library

```
AGENTS.md, CLAUDE.md           short, broad agent guidance (done 2026-10-01)
docs/design/
  README.md                    register: realm table, one line per ID series (range | home | status), how to cite and sign
  INDEX.md                     "if you're here to do X, read Y"
  00-charter.md                section 1                                           (lead)
  01-engine-model.md           section 2                                           (lead; built by Core)
  02-realms.md                 section 3                                           (lead)
  consumers.md                 KAT relationship; the only place KAT rule series may appear
  backlog.md                   cross-realm work: what isn't built, why, what proves it done
  plans/                       lead's plans and specs (this file)
  core/ physics/ rendering/ interface/ test-docs/
      README.md                read order, "here to do X", series owned vs quoted
      00-decisions.md          ID | ruling | signed | status | provenance (the only home for decision status)
      01-…-*.md                the realm's specs, by subject; numbered means normative
      07-status.md             specified vs built, plus open items (needs user / debt); figures carry commit and build tree
      plans/                   the realm's active plans, dated
      08-lessons.md            optional
      superseded/              only when there is something to keep
  superseded/
      kat-originals/           the seven KAT-era originals, moved unedited
      2026-09-consolidation/   today's 01–08, README, RECONCILIATION, pivot-audit, plan-c-sandbox, interleaving-shuffle, sensor-arena-dedup
```

- **IDs.** Every legacy ID (`D`, `A`, `P`, `SA`, `RS`, `SL`, `SR`) gets exactly one home row in one realm's `00-decisions.md`. A split ruling such as `SL6` is listed in both realms, each covering its own half. New rulings use `CORE-n`, `PHY-n`, `RND-n`, `INT-n` and `TD-n`; the laws are `L1`–`L8`. KAT's series (C5, TS, HS, B, G, CP, …) appear only in `consumers.md`.
- **Three rules:** one fact lives in one place; decision status lives in `00-decisions`; current state lives in `07-status`. Everything else is judgement, caught in review.
- **No machine checks** of the library. Moving `docs/v1-transfer-register.md` (and its existing test) under `test-docs/` is optional.
- **Out of scope:** a MkDocs site. `tasks/` stays local scratch.

## 5. KAT separation and transition

**Separation.**
1. The 30 `AgreementMatrix` cases and the scene-drift guard that read `../KAT` move to KAT as consumer tests. Spade's agreement bands are rebuilt on Spade's own content (Rendering with Test/Docs).
2. Remove the `katsrc` remote and the `spade-extract` branch it left behind.
3. Remove or reword KAT references in `scripts/`, `CMakePresets.json`, `.gitignore`, `tests/bench/baselines.json` and the pre-push guard comments (Test/Docs).
4. Rewrite `README.md`, `CHANGELOG.md` and `CONTRIBUTING.md` in Spade's terms: a general quickstart, and a changelog that records the split, GNSS and the GPU rasterizer.
5. Each realm sweeps the KAT-citing comments in its own code (about 60 lines) while it builds its library.
6. Content such as `quad_hover` and `gate` stays, as example content that belongs to the quadrotor template.

**Pause.** SPH (Plan B), wiring in the GPU rasterizer, and sandbox tasks C3–C8 wait until the library exists and the charter and engine model are signed. Each is then redesigned in engine-model terms: SPH as a field provider, the GPU rasterizer as a technique, the sandbox as the editor. **Exception, user-directed 2026-10-01:** the sandbox drone sim box (separate design, `2026-10-01-drone-sim-box-design.md`) proceeds alongside this cleanup.

**Confirmed defects.** Fixes need builds, so they are scheduled one at a time.

| # | Defect | Owner | Handling |
|---|---|---|---|
| 1 | The Vulkan step skips attached behaviors silently, instead of refusing (`simulation.cpp`, Vulkan branch of `step`) | Core | Refuse now; the scheduler redesign removes the cause |
| 2 | The GPU step recorder counts 8+S dispatches per substep but emits 9+S since GNSS, so the last `substeps` barriers are dropped (`step_recorder.cpp`) | Core | Fix by deriving the count from what is emitted, plus a test that pins the count |
| 3 | A `SPADE_VULKAN=OFF` install ships `sim/simulation.hpp`, which includes `compute/` headers that are installed only when Vulkan is on | Core / Interface | Fix |
| 4 | GL lights with `-sun_direction` while the CPU uses `+sun_direction`, and the default sun direction points below the horizon under the documented convention | Rendering | Settle the convention first, then fix |

**Order of work.**
1. **Lead:** the top-level docs and the move into `superseded/`. The user reviews and signs the charter and engine model.
2. **Realms, in parallel, docs only:** each builds its library — legacy IDs given homes, specs rewritten in engine-model terms, `07-status` verified against the tree.
3. **Cleanup and baseline:** the separation items above, plus one measured test count on both presets. No build tree exists in this checkout today.
4. **Afterwards (out of scope here):** each realm's real work gets its own spec and plan, Core first (module API, scheduler, regions).

**Commits.** Each realm commits only its own paths, the lead reviews across realms, and nothing is pushed without the user's word.
