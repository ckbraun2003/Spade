# Spade — cross-realm backlog

What isn't built yet, why, and what proves it done. One row per item. A realm's own debt lives in its `07-status.md`; this file holds what crosses realms or waits on a decision. Remove a row when its proof lands, and say so in that commit.

## Suggested order (lead, 2026-10-02)

1. **Test/Docs: the Docker CI leg** (`TD-11`). It is small, it closes the consumer-smoke gap, and it gates every golden regeneration below (`TD-12`).
2. **Quick wins that need no new architecture:** the GNSS golden (`PHY-6`, built `0ce4ff5`), the v1 baselines (`INT-4`, done `0935c6f`), then the `Lighting{}` regeneration once the Docker leg exists (`RND-5`). Rendering's GL parity work is done: back-face culling (`ec973ed`) and `RenderOptions` (`0a5e1a1`); GL still draws no shadows or overlays.
3. **Core: the engine model's foundation** — module API, scheduler phases, regions, field registry, grades — with **the translation lock** inside it, which lifts the drone sim box's Vulkan refusal.  Stage 1 of 6 merged (`062e1fa`).
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
| World and scene | Mirror Kat. A world holds physics, environment, regions and static terrain. A scene holds a world reference plus placed objects (assets, vehicles, start poses). This changes the engine model's terms and amends `INT-3` to "the editor saves scenes". Both edits land with the joint spec | `01-engine-model.md`, `interface/00-decisions.md` |

No user decision is open.

## Work items

| Item | Owner | Why it's open | Done when |
|---|---|---|---|
| Docker CI leg (`TD-11`, `TD-12`) | Test/Docs, with Interface for the consumer | No CI and no second toolchain since the split. `tests/consumer` was run by hand on 2026-10-02 for the `SPADE_VULKAN=OFF` install (`3a48c4d`); Core's local `tasks/core-offtree.ps1` is a starting point | One script builds the Linux/gcc image, runs `ctest -L spade -LE gpu` on release, and builds and runs `tests/consumer` against an installed prefix with Vulkan ON and OFF |
| GNSS golden scenario (`PHY-6`) | Physics | Built: `gnss_tumble`, merged at `0ce4ff5`. Its digest is provisional under `TD-12` | The Docker gcc leg reproduces the `gnss_tumble` digest. Then remove this row |
| Default-sun regeneration (`RND-5`) | Rendering | `render::Lighting{}`'s default sun is below the horizon | Default flipped; four frame goldens regenerated with provenance, cross-checked by the Docker leg |
| Module API, scheduler phases, regions, field registry, grades | Core (then Physics, Rendering) | The engine model (`01-engine-model.md`) is signed but not built; today's schedule is a fixed ten-pass array | Today's passes run as built-in modules through the new scheduler, with every golden unchanged |
| Translation-lock constraint on both backends | Core / Physics | The drone stand pins with CPU behaviors, so Vulkan is refused there | The drone sim box runs on Vulkan with its position held |
| SPH fluid as a field provider (`PHY-4`) | Physics | The one open v1 transfer row (`docs/v1-transfer-register.md`); v1 stays in the tree until it closes | Row closed with a v2 implementation and a declared grade |
| GPU rasterizer as a render technique (`RND-4`) | Rendering | Stages 1–2 exist but are in no build graph | Built, tested, selectable as a technique, with colour guarded |
| Field channels for cameras | Rendering / Core | The drone heatmap draws a field with sandbox-side cells; the engine has no field channel | A camera renders a registered field as a channel on both CPU and GPU paths |
| Sandbox → editor (`INT-3`) | Interface | The sandbox grows into the editor | An editor spec, approved, then built |
| Spade-owned content for render agreement bands | Rendering / Test/Docs | The bands were measured on KAT's worlds, which left Spade's suite on 2026-10-01 | Bands re-measured on worlds in this repo |
| `SR-3`'s content | Rendering | Every cited `SR-nn` now has a row; `SR-3`'s ruling text could not be reconstructed | Its content is found or the citation is retired |
