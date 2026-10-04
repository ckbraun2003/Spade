# The two raster defects the agreement matrix found: plan

**Owner:** Rendering. **Status:** ruled. A2 is signed as `RND-6` (user, 2026-10-04), and A1 is in the backlog. B1 with B3's warning is built on `rendering/csg-cell-size`: 0.05 m cells, capped at 160 per axis (the lead, 2026-10-04). B2 is being planned by the user's ruling of 2026-10-04. Both defects are debt rows in `../07-status.md`, found by the agreement matrix's first framings (`2026-10-03-agreement-bands-plan.md`). Each fix touches a signed ruling or golden output, so this plan states the root cause and the options before any code.

## Defect A: an infinite heightfield is drawn only within the world bounds

**Root cause.** This is no bug in the code: it is the design. `PA-5` tessellates unbounded primitives (plane, heightfield) as a grid fitted to the world bounds. `SR-17` then draws the infinite remainder analytically, but "standalone planes only". A heightfield has no such pass, so past the bounds the raster shows sky where the SDF, and so collision, has terrain. `RS4` cannot hold there. On `maximal`, the first framings measured d of 7% to 22%.

**Options.**
- **A1. An analytic heightfield background, the way `SR-17` does planes.** The background pass sphere-traces each standalone heightfield leaf for every pixel, and shades the hit like the analytic ground: lambert, grid and horizon term. The tessellated patch still draws over it inside the bounds, as planes do today. Two things differ from planes:
  - **No bit-identical seam.** The patch is Gouraud-shaded per vertex and the background per pixel, so the seam agrees to a band, not to the bit.
  - **Cost.** A sphere trace costs about tens of SDF steps per ground pixel on the CPU. The GL background shader needs the same trace.
  It amends `SR-17`'s "standalone planes only", so it needs the user's signature.
- **A2. A grade note.** The raster draws a heightfield only within the world bounds, and `RS4` holds there. It costs nothing, and it is honest. Heightfields stay limited on the raster until A1.

**Recommendation:** A2 now, as a signed note, and A1 as a backlog item. Today only `maximal`, a test world, uses a heightfield. A1 belongs before terrain reaches real scenes.

## Defect B: CSG meshing loses walls about one cell thick

**Root cause, measured.** `csg_mesh` runs surface nets: corners sampled on a fixed 48-cell grid per subtree, one vertex per active cell, and one quad per sign-changing grid edge. A faithful port to numpy, run on `shower`'s shell (sphere r 4 minus sphere r 3.8 at y 0.4, cell 0.169 m), shows:

| Shape | Cells | Folded triangles (geometric normal against the SDF gradient) | Where |
|---|---|---|---|
| `shower`'s shell | 48 | 648 of 26 780 (2.4%) | y 0.63 to 1.87, where the wall is 0.17 m to 0.03 m |
| control: a shell with an inner radius of 3.0 | 48 | 0 | none |
| `shower`'s shell | 96 | 1 404 (1.2%) | y 1.33 to 2.11, the rim |

Where both faces of a wall cross one cell, their sheets share that cell's single vertex, and quads from the two sheets fold. A folded quad faces backwards and is culled, which makes the holes seen from inside. Where a wall falls between grid samples, it is never sampled at all. **Doubling the resolution halves the folds and moves them up the rim. It cannot remove them, because the rim tapers to zero thickness.**

**Options.**
- **B1. A fixed world-space cell size instead of 48 cells per subtree.** For example, 0.05 m, with a cap on cells per axis. It is still a fixed table (`RS3`), and detail stops depending on a subtree's size. It cuts the defect on walls thicker than about two cells and leaves tapered edges ragged. It moves both CSG goldens, so it follows `TD-1` and `TD-12`. The cost is load time and memory: an 8 m subtree at 0.05 m is 162³ field samples.
- **B2. Ray-march CSG subtrees in the raster instead of meshing them.** Each pixel in a subtree's screen bounds ray-marches that subtree and writes colour and depth. This is exact (`RS4` with d near 0), but every frame pays per pixel, and GL needs the same in a shader. It replaces `RS3`'s "CSG meshed at load", so it needs the user's signature.
- **B3. An authoring rule plus B1:** CSG walls are at least two cells thick. `validate_world` cannot measure wall thickness in general, so the rule would be stated and tested, not enforced.

**Recommendation:** B1 now, with B3's rule stated, and B2 considered if CSG becomes common in real scenes. B1 needs no new ruling, only a golden regeneration through the Docker leg.

## Steps, once the options are chosen

**Defect B (B1):**
1. Write a failing test. Mesh `shower`'s subtree and assert that no triangle's geometric normal opposes the SDF gradient at its centroid, outside a tapered rim region defined with its reason. It fails today with about 648 folded triangles.
2. Make the fix: a world-space cell size in `kCsgMeshDefaults`.
3. Regenerate the CSG goldens. MSVC hashes are held pending, the Docker leg reproduces them, then they are promoted (`TD-1`, `TD-12`).
4. Re-measure the agreement matrix. If `shower`'s inside view now discriminates, it can return as a new band version.

**Defect A:** A2 is a docs change once signed. A1 gets its own plan.

## Questions for the lead

- **Q1.** Defect A: should I propose A2 to the user as a signed note, and file A1 in the backlog?
- **Q2.** Defect B: B1 with B3's rule? What cell size and cap should it use? I'd measure load time at 0.05 m and at 0.08 m before choosing.
