# Core — decisions

**Owner:** Core. The only place a Core decision's status lives. Each legacy ID keeps its original text in the source given; this table is its home row. A ruling marked `signed` was signed by the user on that date. Statuses: `live`, `superseded by X`, `repealed <date>`, `record` (a fact about the past, nothing to obey).

## Legacy rulings homed in Core

| ID | Ruling | Signed | Status | Source |
|---|---|---|---|---|
| `engine D2` | fp32 on both paths with pinned reduction order; worlds are local-frame | approved 2026-08-08 | live, amended by the charter's numerics note (a module may declare fp64) | `superseded/2026-09-consolidation/02-engine.md` §1 |
| `engine D6` (interface half) | Every medium sits behind one sampling interface, so a flow field can replace a provider without touching its readers | approved 2026-08-08 | live, carried into the field model (`03-world-objects-regions.md`). Providers: Physics | `02-engine.md` §1 |
| `engine D7` | One construction path: `WorldBuilder` plus a versioned YAML world file that serializes exactly what it builds | approved 2026-08-08 | live, amended by `RS5` (schema v2) | `02-engine.md` §1 |
| `engine D8` | World-index batching in every buffer from day one; N = 1 is the degenerate case; the CPU steps worlds serially | approved 2026-08-08 | live; restated as `L8` | `02-engine.md` §1, §5 |
| `engine D9` | Shared layouts are single-sourced in Slang and checked by generated `static_assert`s and a generated binding registry | approved 2026-08-08 | live. Codegen tooling: Test/Docs | `02-engine.md` §1, §9 |
| `engine D12` (error half) | `std::expected` is the error spine; no exception crosses the public boundary | approved 2026-08-08 | live. Toolchain half: Test/Docs | `02-engine.md` §1, §10 |
| `engine A1` (schedule half) | Camera-sensor synthesis runs inside stepping, in the sensor phase, never on a presentation queue | signed 2026-08-10 | live. Camera half: Rendering | `02-engine.md` §8 |
| `engine A3` | Reset is restore plus `reseed(seed)`, which re-derives every stream from a new world seed | signed 2026-08-10 (user-ruled) | live. Host half: `../consumers.md` | `02-engine.md` §5 |
| `engine A4` | Per-sensor cursor state is registered and snapshotted; `poll(sensor, since_index)` is stateless in the engine | signed 2026-08-10 | live | `02-engine.md` §5 |
| `engine A7` | Configuration in three tiers: hashed simulation content, a result-neutral backend block (each knob owes an invariance test), and the not configurable | signed 2026-08-10 | live | `02-engine.md` §9 |
| `engine A9` | (1) The tick advances once per step, in `step()`. (2) The Gravity pass is inert; gravity is applied in Integrate | signed 2026-08-10 | (1) live. (2) live until the scheduler lands, then superseded by the engine model (gravity is a field) | `02-engine.md` §4 |
| `charter P1` | Every authoritative pass has a GPU version and a CPU reference twin | approved 2026-08-06 | superseded by `L3`/`L4` (only reference grade needs a CPU twin) | `superseded/2026-09-consolidation/01-charter.md` §2 |
| `charter P2` | Three determinism grades: CPU bit-identical per platform, GPU per device and driver, CPU↔GPU banded | approved 2026-08-06 | superseded by `L3`/`L4` (module grades) | `01-charter.md` §2, §4 |
| `charter P3` | The GPU path is Vulkan: headless compute, explicit synchronization | approved 2026-08-06 | live. Its GLFW sunset clause is stale (`SL14a`) | `01-charter.md` §2 |
| `charter P4` | Fixed step only; no wall clock in the step path; seeded, domain-separated RNG; immutable `dt` | approved 2026-08-06 | superseded by `L1` | `01-charter.md` §2 |
| `charter P8` | Snapshot, save and restore are first-class | approved 2026-08-06 | superseded by `L2` | `01-charter.md` §2 |
| `charter SA3` | Conformance is graded by the three determinism grades of charter §4 (an amendment to KAT's errata) | approved 2026-08-06 | superseded by `L3` | `01-charter.md` §3 |
| `SL3` | The object graph never affects buffer layout, pass order or any computed value; it is not registered state | signed 2026-09-17 | live | `superseded/2026-09-consolidation/04-objects.md` §1 |
| `SL4` | Objects are generational handles with names and transforms; components hold configuration and slot references; attach and detach are structural | signed 2026-09-17 | live. Built differently: attach/detach are immediate; serialization is by name, not id (`07-status.md`) | `04-objects.md` §2 |
| `SL5` | Component type ids are hand-assigned, dense and never renumbered; build only the types in use | signed 2026-09-17 | live | `04-objects.md` §3 |
| `SL6` | Behaviors run in two fixed slots (kinematic before forces, force after built-in elements), registration order is execution order, and a CPU-only behavior is a GPU refusal | signed 2026-09-17 (user ruling 2026-08-30) | live until the scheduler lands; then the slots become phase positions (`01-modules-and-scheduler.md`). Both halves are Core's | `02-engine.md` §4, `04-objects.md` §6 |
| `RS5` (format half) | World schema v2: the file gains `materials`, `lighting`, `props` and a parallel per-node material array; v1 still loads; the writer emits v2 | signed 2026-09-17 | live; **split**: Core owns the format and its version, Rendering owns what the fields mean | `superseded/2026-09-consolidation/03-world-and-render.md` §5 |
| `D-S5-1` | The S1 `ecs/` scaffold is deleted, superseded by `state/` arenas | — | record | `superseded/2026-09-consolidation/07-status.md` §2 |

## Core rulings

| ID | Ruling | Signed | Status | Source |
|---|---|---|---|---|
| `CORE-1` | A Vulkan `step()` with a non-empty behavior registry attached returns `Code::unavailable` and steps nothing; it never skips the behaviors | — | live (merged `ceb4aef`, 2026-10-02) | `plans/2026-10-01-spade-restructure-design.md` §5, defect 1 |
| `CORE-2` | The GPU recorder places a barrier between every adjacent pair of dispatches it actually emitted and none after the last; no hand-kept dispatch count | — | live (merged `c7a36a4`, 2026-10-02) | restructure design §5, defect 2 |
| `CORE-3` | The gaussian draw is CPU↔GPU banded at its source, the Box-Muller `sqrt` in `rng` / `rng.slang`. Vulkan allows ≤ 2.5 ulp where IEEE 754 mandates correct rounding; `log32`/`sin32`/`cos32` and the integer state are bit-identical. Measured: max abs `4.76837158e-07`, max rel `5.06893741e-07`, 11 of 64 ring elements outside a zero band, on an Intel Iris Plus Graphics (Vulkan 1.3.215, driver 31.0.101.2125, msvc-ninja-release), isolated through a GNSS receiver (200 substeps, `bias_tau_s = 1e-6`, `mount_pos = 0`). Scope is by call: `dryden.slang`, `sensor_gnss.slang`, `sensor_imu.slang`. Downstream bands (Physics) cite this one rather than re-deriving a cause. Tightening is refused, because the platform cannot keep it. `GnssDrawsDivergeAcrossBackends_KNOWN_OPEN` asserts the divergence exists, so it goes red if anyone tightens the `sqrt`. The CPU-path goldens are unaffected | — | live (declared 2026-09-24; the measurement is also recorded in `rng.slang`'s header) | `superseded/2026-09-consolidation/01-charter.md` §4.1 |
