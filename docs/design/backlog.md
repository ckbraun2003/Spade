# Spade — cross-realm backlog

What isn't built yet, why, and what proves it done. One row per item. A realm's own debt lives in its `07-status.md`; this file holds what crosses realms or waits on a decision. Remove a row when its proof lands, and say so in that commit.

## Suggested order (lead, 2026-10-02)

1. **Core: the engine model's foundation** — module API, scheduler phases, regions, field registry, grades. Everything below either builds on it or is easier after it.
2. **The translation lock**, inside that work — it lifts the drone sim box's Vulkan refusal.
3. **Physics: SPH as a field provider** — closes the last v1 transfer row.
4. **Rendering: field channels and GL parity** (back-face culling, honouring `RenderOptions`), then the Vulkan raster as a technique.
5. **Interface: the editor spec**, written against the module API once it exists.

Each starts as its own spec and plan in the owning realm's `plans/`.

## Open user decisions

| Decision | Raised by | Where it's recorded |
|---|---|---|
| A second toolchain for the golden cross-check (restore a Linux leg via WSL or a container, or declare goldens MSVC-pinned) | Test/Docs | `test-docs/07-status.md` |
| Engine `D11`'s "own CI legs" with no hosted CI (restore CI, or amend `D11`) | Test/Docs | `test-docs/07-status.md` |
| When SPH (Plan B) resumes | Physics | `physics/07-status.md` |
| When the Vulkan raster resumes as a technique, and whether GL stays as its own technique | Rendering | `rendering/07-status.md` |
| Adopt or delete the unwired Jacobi contact path | Physics | `physics/07-status.md` |
| Regenerate four frame goldens so `render::Lighting{}`'s default sun is above the horizon | Rendering | `rendering/07-status.md` |
| Add a GNSS golden scenario so GNSS reaches reference grade | Physics | `physics/07-status.md` |
| The IMU/GNSS function-pair dedup proposal (or let the module API absorb it) | Core | `core/07-status.md` |
| Saved scenes in the editor's first cut (save/load now, defer, or world-file only) | Interface | `interface/07-status.md` |
| When to capture the `SL14b` v1 baselines before v1 is retired | Interface | `interface/07-status.md` |
| GPU coverage on a one-machine gate: a missing device fails the run, or the run keeps skipping and names its skips | Test/Docs | `test-docs/07-status.md` |

## Work items

| Item | Owner | Why it's open | Done when |
|---|---|---|---|
| Module API, scheduler phases, regions, field registry, grades | Core (then Physics, Rendering) | The engine model (`01-engine-model.md`) is signed but not built; today's schedule is a fixed ten-pass array | Today's passes run as built-in modules through the new scheduler, with every golden unchanged |
| Translation-lock constraint on both backends | Core / Physics | The drone stand pins with CPU behaviors, so Vulkan is refused there | The drone sim box runs on Vulkan with its position held |
| SPH fluid as a field provider | Physics | The one open v1 transfer row (`docs/v1-transfer-register.md`); v1 stays in the tree until it closes | Row closed with a v2 implementation and a declared grade |
| GPU rasterizer as a render technique | Rendering | Stages 1–2 exist but are in no build graph; paused for the restructure | Built, tested, selectable as a technique, with colour guarded |
| Field channels for cameras | Rendering / Core | The drone heatmap draws a field with sandbox-side cells; the engine has no field channel | A camera renders a registered field as a channel on both CPU and GPU paths |
| Sandbox → editor (old plan C tasks C3–C8, rethought) | Interface | The sandbox grows into the editor | An editor spec, approved, then built |
| Spade-owned content for render agreement bands | Rendering / Test/Docs | The bands were measured on KAT's worlds, which left Spade's suite on 2026-10-01 | Bands re-measured on worlds in this repo |
| Out-of-tree consumer smoke | Test/Docs / Interface | `tests/consumer/` is correct but nothing runs it since the split; it was run by hand on 2026-10-02 for the `SPADE_VULKAN=OFF` install (`3a48c4d`) | A script or gate configures, builds and runs it against an installed prefix |
| `SR-3`'s content | Rendering | Every cited `SR-nn` now has a row; `SR-3`'s ruling text could not be reconstructed | Its content is found or the citation is retired |
