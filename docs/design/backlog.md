# Spade — cross-realm backlog

What isn't built yet, why, and what proves it done. One row per item. A realm's own debt lives in its `07-status.md`; this file holds what crosses realms or waits on a decision. Remove a row when its proof lands, and say so in that commit.

| Item | Owner | Why it's open | Done when |
|---|---|---|---|
| Module API, scheduler phases, regions, field registry, grades | Core (then Physics, Rendering) | The engine model (`01-engine-model.md`) is approved but not built; today's schedule is a fixed ten-pass array | Today's passes run as built-in modules through the new scheduler, with every golden unchanged |
| Translation-lock constraint on both backends | Core / Physics | The drone stand pins with CPU behaviors, so Vulkan is refused there | The drone sim box runs on Vulkan with its position held |
| SPH fluid as a field provider | Physics | The one open v1 transfer row (`docs/v1-transfer-register.md`); v1 stays in the tree until it closes | Row closed with a v2 implementation and a declared grade |
| GPU rasterizer as a render technique | Rendering | Stages 1–2 exist but are in no build graph; paused for the restructure | Built, tested, selectable as a technique, with colour guarded |
| Field channels for cameras | Rendering / Core | The drone heatmap draws a field with sandbox-side cells; the engine has no field channel | A camera renders a registered field as a channel on both CPU and GPU paths |
| Sandbox → editor (old plan C tasks C3–C8, rethought) | Interface | Paused for the restructure; the sandbox grows into the editor | An editor spec, approved, then built |
| Spade-owned content for render agreement bands | Rendering / Test/Docs | The bands were measured on KAT's worlds, which leave Spade's suite | Bands re-measured on worlds in this repo |
| Out-of-tree consumer smoke | Test/Docs / Interface | `tests/consumer/` is correct but nothing runs it since the split | A script or gate configures, builds and runs it against an installed prefix |
| Orphaned `SR-nn` citations in code | Rendering (as touched) | Code cites render rulings that were never written down | Each cited `SR-nn` has a row in `rendering/00-decisions.md` |
