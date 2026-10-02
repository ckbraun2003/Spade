# Spade — charter

**Owner:** lead. **Status:** signed by the user 2026-10-02 (approved 2026-10-01; `plans/2026-10-01-spade-restructure-design.md` §1). Changes to the laws need the user's signature.

## Identity

Spade is a general-purpose, deterministic simulation and rendering engine. Developers compose worlds from regions, objects and modules (physics models, sensors, render techniques), then step them headless, batch many at once, or run and inspect them in Spade's editor.

Spade carries no domain concept. Consumers build their domain from modules and templates: KAT for quadrotor training, a vehicle-dynamics tool for F1, and so on. See `consumers.md`.

## Laws

| ID | Law |
|---|---|
| `L1` | Fixed step and seeded. No clock is read in the step path; `dt` is immutable after a simulation is created. |
| `L2` | Snapshot, restore and replay are first-class. Restoring under a different configuration is refused. |
| `L3` | Every module declares a grade: **reference**, **banded** or **best-effort**. A world's grade is that of its weakest module. A consumer may require a minimum grade; a world that cannot meet it is refused, never silently downgraded. |
| `L4` | Reference grade needs a CPU implementation and a golden result. Banded grade needs a measured band against the reference, with a record of how it was measured. The GPU is never a golden source. |
| `L5` | Stepping never depends on rendering. |
| `L6` | No silent fallback. An unavailable backend, module or grade is refused or announced. |
| `L7` | No domain in the core. A proposed addition must be useful to a caller with no interest in that domain. |
| `L8` | Batching many worlds is native. |

**Numerics (provisional):** fp32 is the default for reference grade; a module may declare fp64 as part of its grade.

## What changed from the KAT-era premises

| Was | Now |
|---|---|
| Every authoritative pass has a CPU twin | Only reference-grade modules need one (`L3`, `L4`) |
| State arrays are frozen | Modules register their own state |
| A fixed ten-pass schedule | The scheduler composes module passes into phases (`01-engine-model.md`) |
| "Thin adapter above" (KAT's C5/dronesim) | A consumer relationship, in `consumers.md` |
| Effects layer with an aero roadmap | Fidelity tiers |

The KAT-era charter is `superseded/2026-09-consolidation/01-charter.md`; its rulings keep their IDs and are homed in the realm registers.

## Legacy rulings homed here

| ID | Ruling | Status |
|---|---|---|
| `SL1` | Spade is a domain-neutral engine; KAT is one consumer | superseded by Identity and `L7` (2026-10-01) |
| `SL17` | The 24th spec's phase table P0–P7 (sequencing record) | record only; sequencing now lives in `backlog.md` and realm `plans/` |
