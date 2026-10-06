# Rendering — status

**The only place that says what exists today.** **Re-checked against `master` `13e27ea` on 2026-10-03.** Since the check at `c94add9`, the only render-code changes are this realm's merges `ec973ed`, `1cac77a`, `0a5e1a1`, `875b040` (comments), `3605ddf`, `c0fa53d`, `a46bb86`, `da2fcf5` and `13e27ea`. The user's 2026-10-02 rulings (`RND-4`, `RND-5`, `TD-13`) are folded in. Measurements are under "Measured", each with its commit and build tree.

## Specified vs built

| Specified | Built | Where |
|---|---|---|
| CPU raster technique: shaded, wireframe, ray-march and velocity modes, materials, sun, ambient, sky, shadows, overlays | **Built** (S7a, `9778260`, 2026-09-02) | `engine/render/` → `spade::render`, installed |
| Scene representation (`RS2`, `RS3`): tessellation, CSG subtrees (ray-marched in shaded and velocity frames, meshed for wireframe and shadows), glTF, analytic ground (`SR-17`) | **Built**; CSG by ray-march since B2 (`d895c32`) | `render/scene.*`, `tessellate.*`, `csg_mesh.*`, `gltf.*` |
| Atmospheric term (`SR-17a`) | **Built**, default strength 0. The sandbox turns it on | `RenderOptions::horizon_blend_strength` |
| Analytic infinite grid | **Built**, default off, shaded mode only. The sandbox turns it on | `RenderOptions::ground_grid` |
| Agreement machinery (`RS4`, `SR-30`) | **Built** | `render/agreement.*` |
| Agreement bands (`RS4`, `SR-30`, `SR-31`, `TD-2`) | **Built**: version 2 is active (`f68e2c1`), 6 cases on the golden worlds `gate`, `maximal` and `shower`, each proved to discriminate by the live probe. B2 brought the CSG cases' d near 0, and `shower`'s inside view returned at d = 0. It is attested bit-identical on msvc-release, msvc-debug and gcc-release and signed off by Test/Docs, with a `TD-4` note: on CSG pixels the bands check the raster's integration, not the march. Version 1 (`da2fcf5`) stays pinned. The heightfield stays out of frame (`RND-6`) | `tests/golden/render/agreement_bands.json`, `tests/test_render_agreement_matrix.cpp` |
| OpenGL raster backend | **Built, partial.** Instanced meshes in shaded, wireframe and velocity modes. It culls back faces in shaded and velocity modes (`SR-13`, `ec973ed`). A background pass draws the sky, and in shaded mode the analytic ground, grid and atmospheric term (`0a5e1a1`). It draws the overlays from the CPU's own lists, `render::overlay_geometry()` (`9f76e23`), and the sun's shadows from the CPU's uploaded static map plus each frame's dynamic casters (`9b15a50`), each banded against the CPU on the RTX 3060 Ti (`TD-13`). CSG subtrees are ray-marched within their boxes in shaded and velocity modes, as on the CPU, at most 64 nodes a subtree (`fa3a8f3`). Raymarch is refused with `unavailable`, and `GlRenderer::unhonoured()` names only that (`L6`). Tested by `GpuGlRenderer.*` (`gpu`, `TD-13`). Optional module behind `SPADE_RENDER_GL`. It stays the editor's interactive path (`RND-4`) | `engine/render_gl/` → `spade::render_gl`; `tests/test_gpu_gl_renderer.cpp` |
| Vulkan raster technique | **Stages 1–2 written, in no build graph.** Background only (sky, analytic ground, grid, horizon). The kernel `raster_background.slang` is not compiled, and no test calls `GpuRasterizer`. **Paused.** It resumes as a technique after field channels and Core's module API (`RND-4`) | `render/raster_gpu.hpp`, `render/vulkan/`, `shaders/kernels/raster_background.slang` |
| Field channel (field-channel plan, Part A) | **Built** (`3605ddf`). A field layer is a world-space slice with one sample per cell and a palette colour map. The CPU and GL draw it unlit, double-sided and depth-tested, and refuse a malformed layer. The caller supplies the samples. **Part B**, sampling a registered field through Core, waits on Core's stage 3 | `render/field_layer.hpp`, `RenderScene::field_layers`; `tests/test_render_field.cpp` |
| Channels: depth, IDs, per-pixel data | **Not built.** Each is its own backlog item | — |
| Camera component, camera sensor, frame pool | **Not built.** `render::Camera` is a per-call value | — |
| Ray-trace technique | **Not designed** | — |
| Frame, tessellation and CSG goldens (`RS13`) | **Built**: 6 frames (the sixth, `csg_slab_and_blob_shaded`, has ray-marched CSG, `5b4f60d`), 7 primitives, 2 CSG subtree meshes | `tests/golden/render/` |
| Render bench | **Built**, recorded not gated | `tests/bench/bench_render.cpp` |
| Render-cadence invariance test (`RS13`) | **Not in this repository** (it was a KAT-side case) | — |

## Open items — needs a user decision

**None.** The user ruled the last one on 2026-10-02:
- `RND-4`: the Vulkan raster resumes after field channels and the module API, and OpenGL stays the editor's path.
- `RND-5`: the default sun is flipped after the Docker leg.

`RND-1`–`RND-5` and the reconstructed `SR` rows are signed (`00-decisions.md`). `SR-3` stays unsigned: its content is unknown.

## Next for Rendering

In the backlog's order (`../backlog.md`, "Suggested order"):

1. **The fold warning tells a sharp CSG edge from a thin wall** (Kat's ask, debt row below). Small.
2. **Airflow rendering** (`../physics/plans/2026-10-05-airflow-design.md`, section 5): volume rendering of the published V and S textures, streamlines and smoke, CPU reference first with a synthetic-volume golden, then GL by readback. The bar is research grade (the user, 2026-10-05). The GL CSG march's uniform-buffer follow-up can ride with it, since both use a box proxy.
3. **The field channel, the rest.**
   - Interface moves the drone heatmap onto `field_layers` (plan step 3).
   - Part B binds Core's stage 3 sampling.
4. **The Vulkan raster as a technique** (`RND-4`), after field channels and the module API, with colour guarded before it is selectable.

## Open items — debt

| Item | Detail |
|---|---|
| **Installed headers that cannot compile** (found by Core) | `render/vulkan/background_pass.hpp` includes `compute/vulkan/context.hpp`, and through it volk. `install(DIRECTORY render/ ...)` installs it in every configuration, so a consumer that includes it fails to compile. It is latent while nothing includes it. When the Vulkan raster returns as a technique, either exclude `render/vulkan/` from the install or keep that header Vulkan-free |
| **GPU colour unguarded** | The Vulkan raster's only evidence is coverage. A colour oracle is required before it is selectable (`03-verification.md`). GL's background and field layers now have one each: `GpuGlRenderer.BackgroundMatchesTheCpuWithinItsMeasuredBand` and `GpuGlRenderer.FieldLayerMatchesTheCpuWithinItsMeasuredBand`, regression guards pinned on one device (see "Measured"). GL's mesh shading has none, and differs by design: GL lights per pixel, the CPU per vertex (`SR-18`). Every GL or Vulkan render test is `gpu`-labelled. **Under `TD-13`:** these tests run on the development box, a skip is reported by name and does not count toward a green gate, and the Docker leg excludes them with `-LE gpu` |
| **An infinite heightfield is raster-drawn only within the world bounds** (found by the agreement matrix, 2026-10-03) | `PA-5` tessellates unbounded primitives as a grid fitted to the world bounds. Planes get `SR-17`'s analytic background beyond it; heightfields get nothing. So past the bounds the raster shows sky where the SDF, and collision, have terrain, and `RS4` cannot hold there. On `maximal` the first framings measured d of 7–22%. The matrix keeps the heightfield out of frame and says nothing about it. Accepted, not closed: `RND-6` (signed 2026-10-04) makes it a grade note. The analytic heightfield pass (A1) is in the backlog, to land before terrain reaches real scenes |
| **B1's fine CSG mesh costs wireframe frames and scene load** (found by the B2 bench, 2026-10-04; narrowed by B2) | At 0.05 m, `shower`'s 8 m shell meshes to about 320 000 triangles, against 27 000 at the old 48 cells, and the CPU raster pays about 0.35–0.65 µs a triangle. Since B2, shaded and velocity frames march CSG and skip its mesh, so the cost falls on wireframe frames and on scene load (meshing and the static shadow map). Q2 kept B1's mesh (`0740b34`): 48 cells leaks about 3.5x the shadow (`CsgRaster.ShadowsOnMarchedSurfacesStayCloseToTheTruth`). Open if a consumer draws large CSG in wireframe |
| **Each CSG subtree's program copies the world's whole transform table** (noted in B2 step 3's review, 2026-10-05) | `csg_subtree_program()` keeps every transform so the nodes' indices still resolve, so host memory grows as subtrees x transforms x 80 B. GL uploads only the transforms each subtree uses (`fa3a8f3`), so only host memory pays. Harmless at today's sizes |
| **GL's CSG march cost scales with nodes times grazing steps** (found by B2 step 3, 2026-10-05, `rendering/b2-gl`) | At 1920x1080 on the RTX 3060 Ti, march against mesh: 2.0 against 0.12 ms for small subtrees with shadows, 3.9 against 0.17 ms for a slab filling the frame, and 29.6 against 0.14 ms for a 25-node box cut by 12 holes, filling the frame. Rays grazing down the holes take many steps, and each step evaluates every node from a storage buffer. The follow-up is the program in a uniform buffer and a per-subtree step budget. Not done now; worth doing with the airflow volume pass, which shares the box proxy |
| **The fold warning calls a sharp CSG edge a thin wall** (asked by Kat via the lead, 2026-10-05) | `find_folded_triangles()` flags any triangle whose normal opposes the SDF gradient, and the warning always reads it as a wall thinner than two cells. circuit-track's node 19, a torus with a 0.24 m tube intersected with a box cut at y = 1.6, folds 4 of 42 376 triangles at the sharp cut edge, though the tube is about 4.8 cells thick, so "make it at least 0.100 m thick" is the wrong advice. A crease, where the gradient flips, folds at any thickness. The fix: for each folded triangle, measure the solid's thickness inward along the gradient; under about two cells is a thin wall ("thicken it"), otherwise a sharp edge ("expected, or soften it with a smooth op"). Planned after B2 step 6 |
| **No render-cadence invariance test** | It holds by construction. A Spade-side test is owed (`RS13`) |
| **`SR-17` clause 6** | The infinite ground is unshadowed beyond `scene.bounds`. Accepted, not closed |

## Closed

- 2026-10-02, `94deae3`: **GL lit from below** (restructure defect 4). GL shaded with `dot(n, -sun_direction)`; it now uses `+`, as every other path does (`RND-1`).
- 2026-10-02, `ab2a21e`: **the suite's reach into `../KAT`.** The 30 KAT-world agreement cases, the bookmark-drift guard, both `SPADE_KAT_*` cache variables and the two data files only they read are gone. Three `AgreementProbe` cases were added. The KAT-citing comments in render code were swept in the same merge.
- 2026-10-03, `ec973ed`: **GL did not cull back faces** (`SR-13`). `GlRenderer` now culls, with CCW as the front face, as the CPU does. `GpuGlRenderer.CullsAReversedTriangle` failed before the fix and passes after. The lead's window check found no geometry lost in the drone or builder scenes.
- 2026-10-03, `1cac77a`: **normals under a per-axis scale.** Both paths now take the inverse-transpose, through the cofactor matrix. The CPU keeps `mat3(m)` for conformal matrices, so every rigid frame is bit-identical and no golden moved. `RenderShading.TransformedNormal*` and `GpuGlRenderer.LightsANonUniformlyScaledSurfaceByItsTrueNormal` prove it.
- 2026-10-03, `0a5e1a1`: **GL ignored `RenderOptions`.** See the OpenGL row above. Six `GpuGlRenderer` cases and one host case (`GlRendererOptions`) failed before and pass after.
- 2026-10-03, `875b040`: **stale comments.** `render/shadow.cpp` and `tests/test_render_shadow.cpp` said `LightingDesc`'s default sun is `(0,1,0)`; they now say which suns reach the near-vertical guard.
- 2026-10-03, `3605ddf`: **no field channel.** Field layers are built (see the field channel row above). Three `RenderField` draw cases and the GL band case failed before and pass after. The drone heatmap's 11 cases pass unchanged.
- 2026-10-03, `c0fa53d`: **non-finite field samples drew as bin 0** (the lead's review of `3605ddf`). A NaN or infinite sample now draws in `FieldColourMap::no_data_colour`, a neutral grey that viridis does not contain, on the CPU and GL. `count_non_finite()` gives the caller a count to show. `RenderField.NonFiniteSamplesDrawInTheNoDataColour` and `GpuGlRenderer.FieldLayerShowsNonFiniteSamplesAsNoData` failed before and pass after.
- 2026-10-03, `a46bb86`: **`render::Lighting{}`'s default sun was below the horizon** (`RND-5`). It is now `normalize(LightingDesc{}.sun_direction)`, by the expression `scene_from_world()` uses, so the two defaults cannot drift. Four frame goldens moved and `sphere_wireframe_no_overlays`, the control, did not; the lead reviewed the before and after frames. The new hashes were measured on MSVC and reproduced in full by the Docker gcc leg on `b82b72e`, so they are promoted (`TD-12`). The manifest's `_changelog_rnd5` and `_pending_rnd5` (discharged) record it. `RasterGolden` gained an opt-in frame dump, `SPADE_RENDER_DUMP_FRAMES`.
- 2026-10-03, `da2fcf5`: **no agreement bands on Spade's own worlds.** `AgreementMatrix` runs version 1 of `agreement_bands.json`: 5 cases, bands 0.012, 0.012, 0.002, 0.004 and 0.005, with probe-over-band 3.8 to 49. The margin is Test/Docs' TD-2 rule, signed off per case. Three first framings were dropped and two reframed after their frames showed two raster defects (debt rows above).
- 2026-10-03, `13e27ea`: **GL's glad globals would clash with a consumer's loader** once `render_gl` is installed (Interface's plan `513e1b8`). `GlRenderer` loads its 45 GL functions into a private table through its `GlProcLoader`, and refuses a missing one by name before any GL call (`GlRendererCreate.ANullLoaderIsRefusedBeforeAnyGlCall`). glad is a private build-tree header only.
- 2026-10-05, `9f76e23`, `9b15a50`: **GL draws no shadows or overlays.** GL now draws both. Overlays come from the CPU's own lists through `render::overlay_geometry()`, with the inverse-depth bias ported to the vertex shader (`SR-21`). Shadows sample the CPU's static map uploaded as a texture, plus dynamic casters drawn into a per-frame copy. Bands are pinned on the RTX 3060 Ti, NVIDIA 572.83 (`TD-13`): overlays 4 CPU misses of 3277, shadows 0. `GlRenderer::unhonoured()` now returns `{}` for `RenderOptions{}`, so the sandbox HUD's list is empty.
- 2026-10-05, `d895c32`, `fa3a8f3`, `0740b34`: **CSG meshing lost walls thinner than about two cells** (defect B). B2 ray-marches each CSG subtree per pixel in shaded and velocity frames, on the CPU and GL (`RS3` as replaced), so a wall of any thickness draws whole: `shower`'s inside view, dropped from the agreement matrix at d = 0.252, returns at d = 0 (bands version 2, `f68e2c1`). The mesh still folds at a thin wall; it draws wireframe and casts the shadow, and the fold warning says so.
- 2026-10-05, `0740b34`: **the fold warning's cell figure ignored a smooth_union root's margin.** It now prints `csg_mesh_cell_size()`, the cell the mesh samples.

## Measured

| Commit | Build tree | ctest `-L spade` (release) |
|---|---|---|
| `5501370` (`rendering/sun-convention`, before merge) | `../spade-wt/rendering/build-ninja/release` | 897 total, 864 passed, 33 skipped, 0 failed. Skips: 31 KAT-dependent cases with no `../KAT` beside a worktree, plus the standing `Fp32Exp.FullDomainSweepEveryFloatArgument` and `SlangLayouts.DeliberatelyUnboundArraysHaveNoBinding`. `gpu` label: 63 tests ran. No frame golden moved |
| `e7e1039` (`rendering/kat-reach`, before merge) | same tree | 869 total, 867 passed, 2 skipped (the two standing skips), 0 failed. 869 = 897 − 31 + 3, as predicted |
| `02c1a96` (`rendering/gl-culling`, before merge) | same tree | 923 total, 921 passed, 2 skipped (the standing two), 0 failed. 67 `gpu` tests ran |
| `c1fc309` (`rendering/gl-render-options`, on `rendering/normals-scale`, rebased on `4d4f7a5`) | same tree | 939 total, 937 passed, 2 skipped (the standing two), 0 failed. 77 `gpu` tests ran. 939 = 929 on `4d4f7a5` + 10 |
| `de30813` (`rendering/field-channel`, rebased on `afdbdf5`) | same tree | 978 total, 976 passed, 2 skipped (the standing two), 0 failed. 78 `gpu` tests ran. 978 = 971 on `afdbdf5` + 7 |
| `5685c7c` (`rendering/field-no-data`, rebased on `aeaef92`) | same tree | 988 total, 986 passed, 2 skipped (the standing two), 0 failed. 79 `gpu` tests ran. 988 = 986 on `aeaef92` + 2 |
| `841bf41` (`rendering/default-sun`, commit B, on `aeaef92`) | same tree | 986 total, 984 passed, 2 skipped (the standing two), 0 failed. 78 `gpu` tests ran. With the flip and the new manifest, no test other than the four regenerated frames moved |
| `324d152` (`rendering/gl-private-loader` on `rendering/agreement-bands`, on `4c511b4`) | `../spade-wt/rendering/build-ninja/release` | 998 total, 996 passed, 2 skipped (the standing two), 0 failed. 82 `gpu` tests ran. 998 = 992 on `4c511b4` + 5 agreement cases + 1 loader case |
| `b82b72e` (commit B, rebased on `fa33656`) | **Docker gcc leg**, `build-docker/b82b72ede453` | 908 run, 906 passed, 2 skipped (the standing two), 0 failed; 79 `gpu` excluded (`TD-13`). gcc-13.3.0, cmake 3.28.3, Release. Every `RasterGolden` case passed against B's manifest, so the four RND-5 hashes reproduced on a second toolchain. The consumer smoke passed with Vulkan on and off |

These were measured on the branches, not on the merged `master`, which also carries other realms' merges. `test-docs/07-status.md` holds the suite's baseline.

**GL background band** (`GpuGlRenderer.BackgroundMatchesTheCpuWithinItsMeasuredBand`). Measured at `087c3c3` on Intel Iris Plus Graphics, driver 31.0.101.2125, at 160x120. GL differs from `raster_cpu` by at most 1 level over all 19200 pixels, and by 1 over the 15821 interior pixels (no grid line or horizon within 1 px). Both bands are pinned at 1. Another device may differ; re-measure there before widening.

**GL field-layer band** (`GpuGlRenderer.FieldLayerMatchesTheCpuWithinItsMeasuredBand`). Measured at `a89d8ca` on the same device (GL 4.3), at 160x120. GL differs from `raster_cpu` by at most 1 level over all pixels, and by 1 over the 15960 interior pixels. No cell-border pixel changed cell. Both bands are pinned at 1.

**GL overlay band** (`GpuGlRenderer.OverlaysMatchTheCpuWithinTheirBand`). Measured at `ebe1ba2` on an NVIDIA GeForce RTX 3060 Ti, OpenGL 4.3.0, driver 572.83, at 160x120. 4 of the CPU's 3277 overlay pixels have no GL overlay pixel within 1 px, and 0 of GL's 3282 miss the other way. The 4 are where a bounds edge crosses grid lines. Pinned at 4 and 0. GL biases vertices before clipping and none behind the eye, so a grid line crossing the eye plane bends about 0.2 px against the CPU; that stays inside the band. `OverlayBiasKeepsTheGridOnACoincidentGroundMesh` pins the ported bias itself (`SR-21`).

**GL shadow bands** (`GpuGlRenderer.StaticShadowsMatchTheCpuWithinTheirBand`, `…DynamicCasterShadowsMatchTheCpuWithinTheirBand`). Measured at `b2cf617` on the same RTX 3060 Ti and driver, at 160x120. Both cases show 776 shadowed pixels on each path, 0 misses either way, and 0 levels of difference over the 648 interior pixels. Pinned at 0. GL samples the CPU's own static map, so the static case is exact by construction; the dynamic casters are GL's own raster into a copy of it.

**GL CSG band** (`GpuGlRenderer.CsgMatchesTheCpuWithinItsBand`). Measured at `e11d2d5` on the RTX 3060 Ti, driver 572.83, at 160x120: 2709 covered pixels on each path, 0 misses either way, and at most 1 level apart over the 2335 interior pixels. Pinned at 0 and 1. The scene's tessellated balls are unlit: a lit mesh is Gouraud on the CPU (`SR-18`) and per pixel on GL, which alone moves a lit ball's pixels by up to 11 levels.

## Corrections to earlier records

- 2026-10-01: the consolidation's status said S7b was "NOT STARTED". The Vulkan raster's stages 1–2 had landed (`038b82b`, `018fc09`, `95333c5`).
