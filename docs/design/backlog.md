# Spade — cross-realm backlog

What isn't built yet, why, and what proves it done. One row per item. A realm's own debt lives in its `07-status.md`; this file holds what crosses realms or waits on a decision. Remove a row when its proof lands, and say so in that commit.

## Suggested order (lead, 2026-10-02)

1. **Test/Docs: the Docker CI leg** (`TD-11`). It is small, it closes the consumer-smoke gap, and it gates every golden regeneration below (`TD-12`). **Built** (`44a4925`); its first run (`fe4934a`) was the first `TD-12` cross-check.
2. **Quick wins that need no new architecture:** the GNSS golden (`PHY-6`, built `0ce4ff5`, final under `TD-12` after the first Docker gcc run), the v1 baselines (`INT-4`, done `0935c6f`), then the `Lighting{}` regeneration (`RND-5`, merged `a46bb86`, reproduced on gcc). Rendering's GL parity work is done: back-face culling (`ec973ed`) and `RenderOptions` (`0a5e1a1`); GL still draws no shadows or overlays.
3. **Core: the engine model's foundation** — module API, scheduler phases, regions, field registry, grades — with **the translation lock** inside it, which lifts the drone sim box's Vulkan refusal.  Stages 1 to 3 of 6 merged (`062e1fa`, `dca7cfb`, `42b352a`).
4. **On top of the module API:** SPH as a field provider (`PHY-4`), field channels for cameras, then the Vulkan raster as a technique (`RND-4`), and the editor spec (`INT-3`).

Each starts as its own spec and plan in the owning realm's `plans/`.

## Ruled by the user, 2026-10-02

Every decision this file held is ruled; the rulings live in the realm registers.

| Decision | Ruling |
|---|---|
| CI and the golden cross-check toolchain | Self-run Docker legs, no hosted CI: `TD-11` (amends `engine D11`), `TD-12` |
| GPU coverage on a one-machine gate | `TD-13` |
| When SPH resumes | After Core's module API: `PHY-4` |
| The Jacobi contact path | Kept unwired, returns as a contact module: `PHY-5` |
| A GNSS golden | Now: `PHY-6` |
| The Vulkan raster, and GL's future | After field channels and the module API; GL stays the editor's path: `RND-4` |
| The below-horizon default sun | Flip with one regeneration, after the Docker leg: `RND-5` |
| The IMU/GNSS dedup | Absorbed by the module API: `CORE-4` |
| Editor saves | World files only: `INT-3` |
| The `SL14b` v1 baselines | Capture now: `INT-4` |

## Ruled by the user, 2026-10-03

| Decision | Ruling | Where it lands |
|---|---|---|
| Module-API scope (Core Q1) | Foundation only; regions, objects in stepping and world file v3 are specified, then built later | `core/plans/2026-10-02-module-api-design.md` |
| How responders read fields (Core Q2) | Sample buffers; kinematic behaviors run first in Fields | the same spec |
| The module-API spec | Approved at `4d481eb`, with a requirements section to add | the same spec |
| The engineering style guide | Adopt it now | `../style/engineering-style-guide.md` |
| A joint drone-builder spec with Kat | Write it now, in parallel with the round; the user approves it before anything is built | `plans/2026-10-03-drone-builder-engine-design.md` |
| The joint drone-builder spec | Approved 2026-10-03 (Kat `c9683af3`; Spade's `plans/2026-10-03-drone-builder-engine-design.md` and `physics/plans/2026-10-03-drone-builder-physics.md`). Building starts in the order below | work items below |
| World and scene | Mirror Kat. A world holds physics, environment, regions and static terrain. A scene holds a world reference plus placed objects (assets, vehicles, start poses). This changes the engine model's terms and amends `INT-3` to "the editor saves scenes". Both edits land with the joint spec | `01-engine-model.md`, `interface/00-decisions.md` |

## Ruled by the user, 2026-10-04

| Decision | Ruling | Where it lands |
|---|---|---|
| `RND-6`: the raster draws a heightfield only within the world bounds, and `RS4` holds there | Signed | `rendering/00-decisions.md` (`RND-6` signed), the A2 grade note |
| An analytic heightfield background past the world bounds (A1, amends `SR-17`) | Backlog. Do it before terrain reaches real scenes | work item below |
| Ray-march CSG subtrees in the raster instead of meshing them at load (B2, replaces `RS3`) | Plan it now. The plan comes back for the user's signature on replacing `RS3`. B1 (world-space cell size) lands first regardless | work item below |

## Open user decisions

None open.

## Work items

| Item | Owner | Why it's open | Done when |
|---|---|---|---|
| Ray-marched CSG in the raster and GL (B2) | Rendering | The user ruled on 2026-10-04 to plan it now. Meshing at load (`RS3`) loses walls about one cell thick | A plan, approved, including the `RS3` replacement for the user's signature |
| Analytic heightfield background past the world bounds (A1) | Rendering | Backlog by the user's ruling of 2026-10-04. Past the bounds the SDF and collision have terrain the raster does not draw (`RND-6`) | Before terrain reaches real scenes: a plan, then the `SR-17` amendment signed and built |
| Module API, scheduler phases, regions, field registry, grades | Core (then Physics, Rendering) | The engine model (`01-engine-model.md`) is signed but not built; today's schedule is a fixed ten-pass array | Today's passes run as built-in modules through the new scheduler, with every golden unchanged |
| Translation-lock constraint on both backends | Core / Physics | The drone stand pins with CPU behaviors, so Vulkan is refused there | The drone sim box runs on Vulkan with its position held |
| SPH fluid as a field provider (`PHY-4`) | Physics | The one open v1 transfer row (`docs/v1-transfer-register.md`); v1 stays in the tree until it closes | Row closed with a v2 implementation and a declared grade |
| GPU rasterizer as a render technique (`RND-4`) | Rendering | Stages 1–2 exist but are in no build graph | Built, tested, selectable as a technique, with colour guarded |
| Field channels for cameras | Rendering / Core | The drone heatmap draws a field with sandbox-side cells; the engine has no field channel | A camera renders a registered field as a channel on both CPU and GPU paths |
| Sandbox → editor (`INT-3`) | Interface | The sandbox grows into the editor | An editor spec, approved, then built |
| Spade-owned content for render agreement bands | Rendering / Test/Docs | The bands were measured on KAT's worlds, which left Spade's suite on 2026-10-01 | Bands re-measured on worlds in this repo |
| Drone builder: model functions and mass properties | Physics | Approved joint spec; Kat's airframe fit runs against them | Motor/ESC, battery and propeller pure functions, the composite-inertia utility and the steady-state solver, built and tested before module API stage 4 (DBP-01..52) |
| Drone builder: `spade::scene` | Core (schema), Interface (composer) | Kat sends scenes, and the editor saves them | The scene file loads, round-trips and composes into a runnable world (SCN-001..009); snapshot v3 carries the model-registry identity; `consumers.md` records it |
| Drone builder: stateful motor and battery modules | Physics, with Core | Needs module API stage 4 | Motor/ESC and battery rows step on both backends with their goldens and bands; a duty-command call and a design-frame state read exist (DBE-013) |
| CSG world-space cell size and the two-cell wall warning (B1, B3) | Rendering | csg_mesh's fixed 48 cells per subtree fold or lose walls about one cell thick (`rendering/07-status.md` debt) | A world-space cell size with a cap; a validation warning names any wall thinner than two cells; the CSG goldens regenerated through `TD-1` and `TD-12` |
| `SR-3`'s content | Rendering | Every cited `SR-nn` now has a row; `SR-3`'s ruling text could not be reconstructed | Its content is found or the citation is retired |
