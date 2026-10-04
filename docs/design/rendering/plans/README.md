# Rendering plans

Active Rendering plans, one file each, named `YYYY-MM-DD-<topic>.md`. The drone sim box's Rendering task (the sun convention) is in the lead's plan, `../../plans/2026-10-01-drone-sim-box-plan.md`, Task 3.

- `2026-10-02-gl-render-options.md`: GL honours `RenderOptions` (sky, analytic ground, grid, the atmospheric term, wireframe). Done, merged at `0a5e1a1`.
- `2026-10-03-field-channel-plan.md`: a camera draws a registered field as a channel, on the CPU and GL. Part A is done (merged at `3605ddf`). Step 3 is Interface's. Part B waits on Core's stage 3.
- `2026-10-03-default-sun-plan.md`: `RND-5`, the default sun above the horizon, with four frame goldens regenerated and reproduced by the Docker leg. Done, merged at `a46bb86`.
- `2026-10-03-agreement-bands-plan.md`: agreement bands on Spade's own worlds (`RS4`, `SR-30`, `SR-31`, `TD-2`), replacing the 30 cases that moved to KAT. Done, merged at `da2fcf5` (5 cases; two defects recorded as debt).
- `2026-10-03-raster-defects-plan.md`: root causes and fix options for the two defects the agreement matrix found (heightfield past the world bounds; CSG walls about one cell thick). Ruled: A2 signed as `RND-6`, A1 in the backlog, B1 built on `rendering/csg-cell-size`, B2 being planned.
- `2026-10-04-b2-raymarched-csg-plan.md`: B2, the raster ray-marching CSG subtrees instead of meshing them. It measures the cost against B1's mesh, and proposes the `RS3` replacement for the user's signature. Draft, for the lead's review.
