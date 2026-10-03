# Rendering — status

**The only place that says what exists today.** **Re-checked against `master` `a46bb86` on 2026-10-03.** Since the check at `c94add9`, the only render-code changes are this realm's merges `ec973ed`, `1cac77a`, `0a5e1a1`, `875b040` (comments), `3605ddf`, `c0fa53d` and `a46bb86`. The user's 2026-10-02 rulings (`RND-4`, `RND-5`, `TD-13`) are folded in. Measurements are under "Measured", each with its commit and build tree.

## Specified vs built

| Specified | Built | Where |
|---|---|---|
| CPU raster technique: shaded, wireframe, ray-march and velocity modes, materials, sun, ambient, sky, shadows, overlays | **Built** (S7a, `9778260`, 2026-09-02) | `engine/render/` → `spade::render`, installed |
| Scene representation (`RS2`, `RS3`): tessellation, CSG meshing, glTF, analytic ground (`SR-17`) | **Built** | `render/scene.*`, `tessellate.*`, `csg_mesh.*`, `gltf.*` |
| Atmospheric term (`SR-17a`) | **Built**, default strength 0. The sandbox turns it on | `RenderOptions::horizon_blend_strength` |
| Analytic infinite grid | **Built**, default off, shaded mode only. The sandbox turns it on | `RenderOptions::ground_grid` |
| Agreement machinery (`RS4`, `SR-30`) | **Built** | `render/agreement.*` |
| Agreement bands | **None.** The only bands were measured on KAT's worlds, read from `../KAT`; that matrix and its data left the suite at `ab2a21e` | `../backlog.md` |
| OpenGL raster backend | **Built, partial.** Instanced meshes in shaded, wireframe and velocity modes. It culls back faces in shaded and velocity modes (`SR-13`, `ec973ed`). A background pass draws the sky, and in shaded mode the analytic ground, grid and atmospheric term (`0a5e1a1`). **Not drawn:** shadows and overlays, which `GlRenderer::unhonoured()` names. Raymarch is refused with `unavailable` (`L6`). Tested by `GpuGlRenderer.*` (`gpu`, `TD-13`). Optional module behind `SPADE_RENDER_GL`. It stays the editor's interactive path (`RND-4`) | `engine/render_gl/` → `spade::render_gl`; `tests/test_gpu_gl_renderer.cpp` |
| Vulkan raster technique | **Stages 1–2 written, in no build graph.** Background only (sky, analytic ground, grid, horizon). The kernel `raster_background.slang` is not compiled, and no test calls `GpuRasterizer`. **Paused.** It resumes as a technique after field channels and Core's module API (`RND-4`) | `render/raster_gpu.hpp`, `render/vulkan/`, `shaders/kernels/raster_background.slang` |
| Field channel (field-channel plan, Part A) | **Built** (`3605ddf`). A field layer is a world-space slice with one sample per cell and a palette colour map. The CPU and GL draw it unlit, double-sided and depth-tested, and refuse a malformed layer. The caller supplies the samples. **Part B**, sampling a registered field through Core, waits on Core's stage 3 | `render/field_layer.hpp`, `RenderScene::field_layers`; `tests/test_render_field.cpp` |
| Channels: depth, IDs, per-pixel data | **Not built.** Each is its own backlog item | — |
| Camera component, camera sensor, frame pool | **Not built.** `render::Camera` is a per-call value | — |
| Ray-trace technique | **Not designed** | — |
| Frame, tessellation and CSG goldens (`RS13`) | **Built**: 5 frames, 7 primitives, 2 CSG subtrees | `tests/golden/render/` |
| Render bench | **Built**, recorded not gated | `tests/bench/bench_render.cpp` |
| Render-cadence invariance test (`RS13`) | **Not in this repository** (it was a KAT-side case) | — |

## Open items — needs a user decision

**None.** The user ruled the last one on 2026-10-02:
- `RND-4`: the Vulkan raster resumes after field channels and the module API, and OpenGL stays the editor's path.
- `RND-5`: the default sun is flipped after the Docker leg.

`RND-1`–`RND-5` and the reconstructed `SR` rows are signed (`00-decisions.md`). `SR-3` stays unsigned: its content is unknown.

## Next for Rendering

In the backlog's order (`../backlog.md`, "Suggested order"):

1. **Agreement bands on Spade's own worlds** (`plans/2026-10-03-agreement-bands-plan.md`). The matrix is written; it measures and pins in slot 15.
2. **The field channel, the rest.**
   - Interface moves the drone heatmap onto `field_layers` (plan step 3).
   - Part B binds Core's stage 3 sampling.
3. **The Vulkan raster as a technique** (`RND-4`), after field channels and the module API, with colour guarded before it is selectable.

## Open items — debt

| Item | Detail |
|---|---|
| **Installed headers that cannot compile** (found by Core) | `render/vulkan/background_pass.hpp` includes `compute/vulkan/context.hpp`, and through it volk. `install(DIRECTORY render/ ...)` installs it in every configuration, so a consumer that includes it fails to compile. It is latent while nothing includes it. When the Vulkan raster returns as a technique, either exclude `render/vulkan/` from the install or keep that header Vulkan-free |
| **GPU colour unguarded** | The Vulkan raster's only evidence is coverage. A colour oracle is required before it is selectable (`03-verification.md`). GL's background and field layers now have one each: `GpuGlRenderer.BackgroundMatchesTheCpuWithinItsMeasuredBand` and `GpuGlRenderer.FieldLayerMatchesTheCpuWithinItsMeasuredBand`, regression guards pinned on one device (see "Measured"). GL's mesh shading has none, and differs by design: GL lights per pixel, the CPU per vertex (`SR-18`). Every GL or Vulkan render test is `gpu`-labelled. **Under `TD-13`:** these tests run on the development box, a skip is reported by name and does not count toward a green gate, and the Docker leg excludes them with `-LE gpu` |
| **No agreement bands on Spade's own worlds** | `../backlog.md`. Version 1 (5 cases) is pinned on `rendering/agreement-bands`, not merged yet (`plans/2026-10-03-agreement-bands-plan.md`) |
| **An infinite heightfield is raster-drawn only within the world bounds** (found by the agreement matrix, 2026-10-03) | `PA-5` tessellates unbounded primitives as a grid fitted to the world bounds. Planes get `SR-17`'s analytic background beyond it; heightfields get nothing. So past the bounds the raster shows sky where the SDF, and collision, have terrain, and `RS4` cannot hold there. On `maximal` the first framings measured d of 7–22%. The matrix keeps the heightfield out of frame and says nothing about it. Options: an analytic heightfield pass like `SR-17`'s, or a grade note that the raster draws heightfields only within the bounds |
| **CSG meshing loses walls thinner than about two cells** (found by the agreement matrix, 2026-10-03) | `csg_mesh` runs surface nets on a fixed 48-cell grid per subtree (`kCsgMeshDefaults`, `RS3`). On `shower`, an 8 m bowl whose shell is 0.2–0.6 m thick, a cell is about 0.17 m, so the thin part is about one cell. One vertex per cell cannot hold both faces: the rim is ragged, and from inside the wall shows holes in a checkerboard of culled, mixed-winding quads (d=25%). The matrix views only the thick underside. Options: a finer grid for thin subtrees, a dual method, or a minimum-thickness rule validated at authoring |
| **No render-cadence invariance test** | It holds by construction. A Spade-side test is owed (`RS13`) |
| **GL draws no shadows or overlays** | `GlRenderer::unhonoured()` names both for a caller to show or refuse (`L6`). For `RenderOptions{}` it returns `{"shadows", "overlays"}`. The sandbox HUD shows the list while GL is the path (Interface, `afdbdf5`) |
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
| `b82b72e` (commit B, rebased on `fa33656`) | **Docker gcc leg**, `build-docker/b82b72ede453` | 908 run, 906 passed, 2 skipped (the standing two), 0 failed; 79 `gpu` excluded (`TD-13`). gcc-13.3.0, cmake 3.28.3, Release. Every `RasterGolden` case passed against B's manifest, so the four RND-5 hashes reproduced on a second toolchain. The consumer smoke passed with Vulkan on and off |

These were measured on the branches, not on the merged `master`, which also carries other realms' merges. `test-docs/07-status.md` holds the suite's baseline.

**GL background band** (`GpuGlRenderer.BackgroundMatchesTheCpuWithinItsMeasuredBand`). Measured at `087c3c3` on Intel Iris Plus Graphics, driver 31.0.101.2125, at 160x120. GL differs from `raster_cpu` by at most 1 level over all 19200 pixels, and by 1 over the 15821 interior pixels (no grid line or horizon within 1 px). Both bands are pinned at 1. Another device may differ; re-measure there before widening.

**GL field-layer band** (`GpuGlRenderer.FieldLayerMatchesTheCpuWithinItsMeasuredBand`). Measured at `a89d8ca` on the same device (GL 4.3), at 160x120. GL differs from `raster_cpu` by at most 1 level over all pixels, and by 1 over the 15960 interior pixels. No cell-border pixel changed cell. Both bands are pinned at 1.

## Corrections to earlier records

- 2026-10-01: the consolidation's status said S7b was "NOT STARTED". The Vulkan raster's stages 1–2 had landed (`038b82b`, `018fc09`, `95333c5`).
