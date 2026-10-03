# Rendering — status

**The only place that says what exists today.** **Re-checked against `master` `c94add9` on 2026-10-02** by reading the code, the CMake and the log since the first check (`0dd8bdc`). The only render-code changes since then are this realm's merges at `94deae3` and `ab2a21e`. **Updated at `be4da49` for the user's 2026-10-02 rulings** (`RND-4`, `RND-5`, `TD-13`). No code has changed since `c94add9`. Measurements are under "Measured", each with its commit and build tree.

## Specified vs built

| Specified | Built | Where |
|---|---|---|
| CPU raster technique: shaded, wireframe, ray-march and velocity modes, materials, sun, ambient, sky, shadows, overlays | **Built** (S7a, `9778260`, 2026-09-02) | `engine/render/` → `spade::render`, installed |
| Scene representation (`RS2`, `RS3`): tessellation, CSG meshing, glTF, analytic ground (`SR-17`) | **Built** | `render/scene.*`, `tessellate.*`, `csg_mesh.*`, `gltf.*` |
| Atmospheric term (`SR-17a`) | **Built**, default strength 0. The sandbox turns it on | `RenderOptions::horizon_blend_strength` |
| Analytic infinite grid | **Built**, default off, shaded mode only. The sandbox turns it on | `RenderOptions::ground_grid` |
| Agreement machinery (`RS4`, `SR-30`) | **Built** | `render/agreement.*` |
| Agreement bands | **None.** The only bands were measured on KAT's worlds, read from `../KAT`; that matrix and its data left the suite at `ab2a21e` | `../backlog.md` |
| OpenGL raster backend | **Built, partial.** Instanced meshes with lambert/unlit shading only. It ignores `RenderOptions`: no sky, analytic ground, grid, shadows, overlays, draw modes or atmospheric term (the sandbox clears to a flat colour). Optional module behind `SPADE_RENDER_GL`. It stays the editor's interactive path (`RND-4`) | `engine/render_gl/` → `spade::render_gl` |
| Vulkan raster technique | **Stages 1–2 written, in no build graph.** Background only (sky, analytic ground, grid, horizon). The kernel `raster_background.slang` is not compiled, and no test calls `GpuRasterizer`. **Paused.** It resumes as a technique after field channels and Core's module API (`RND-4`) | `render/raster_gpu.hpp`, `render/vulkan/`, `shaders/kernels/raster_background.slang` |
| Channels: depth, IDs, fields | **Not built.** Colour only | — |
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

1. **GL back-face culling** (`SR-13`).
   - Can run now, alongside the backlog's quick wins.
   - Drone plan Task 7 passed with the builder and drone meshes outward-wound.
   - Proved by GL's first automated test, `gpu`-labelled (`TD-13`).
2. **GL honours `RenderOptions`**: sky, analytic ground and grid at least, so the GL viewport stops drawing on a flat black clear. **Requirement carried from item 1:** when GL gains a wireframe mode, wireframe draws both windings with culling off (`SR-13`).
3. **The default-sun regeneration** (`RND-5`), after the Docker gcc leg exists (`TD-11`, `TD-12`). See the debt row below.
4. **A field channel for cameras**, after Core's module API. It replaces the drone heatmap's sandbox-side cells (`../backlog.md`).
5. **The Vulkan raster as a technique** (`RND-4`), after field channels and the module API, with colour guarded before it is selectable.

## Open items — debt

| Item | Detail |
|---|---|
| **`render::Lighting{}`'s default sun is below the horizon** | `{-0.35, -0.86, -0.37}` points away from the sun under `RND-1`. Four hand-built frame goldens shade under it: `box_shaded_with_overlays`, `cylinder_static_dynamic_box_top_down`, `shadowed_ground_with_caster` and its `_atmospheric` pair. **Ruled by `RND-5`:** flip it with one deliberate regeneration once the Docker gcc leg exists, because a regenerated golden is final only when that leg reproduces it (`TD-12`). World-loaded scenes are unaffected, because `LightingDesc`'s default `(0.4, 0.8, 0.6)` is correct |
| **Normals under non-uniform scale** | Both paths transform normals by `mat3(model)` with no inverse-transpose (`transform_normal()` on the CPU, the GL vertex shader), so lambert shading is wrong on any non-uniformly scaled item. **No golden uses non-uniform scale:** frame goldens are rigid, and props and SDF poses take uniform scale only. So the fix moves no golden. It does affect the sandbox builder today, whose objects take a per-axis scale |
| **Installed headers that cannot compile** (found by Core) | `render/vulkan/background_pass.hpp` includes `compute/vulkan/context.hpp`, and through it volk. `install(DIRECTORY render/ ...)` installs it in every configuration, so a consumer that includes it fails to compile. It is latent while nothing includes it. When the Vulkan raster returns as a technique, either exclude `render/vulkan/` from the install or keep that header Vulkan-free |
| **GPU colour unguarded** | The Vulkan raster's only evidence is coverage. A colour oracle is required before it is selectable (`03-verification.md`). That oracle, and every GL or Vulkan render test, is `gpu`-labelled. **Under `TD-13`:** these tests run on the development box, a skip is reported by name and does not count toward a green gate, and the Docker leg excludes them with `-LE gpu` |
| **No agreement bands on Spade's own worlds** | `../backlog.md` |
| **No render-cadence invariance test** | It holds by construction. A Spade-side test is owed (`RS13`) |
| **GL does not cull back faces** | `SR-13` says shaded mode culls back faces, and the CPU path does. `GlRenderer` never enables `GL_CULL_FACE`. So a mesh wound inward still draws on GL, lit through its inverted normals, while the CPU culls its near faces. That is how the sandbox builder's inward-wound sphere and cylinder went unnoticed on GL: their inverted normals cancelled the inverted sun. Interface fixed the winding (`c03bb94`) before the sun fix merged. **Culling on GL is still owed**, and now unblocked. On correct meshes it is invisible, so anything that vanishes is a real winding bug |
| **GL and CPU frames differ in content** | Because GL ignores `RenderOptions`, a scene looks different on the two paths beyond numerics: no sky, ground or shadows on GL. The sandbox clears to a near-black colour under it |
| **Stale comments** | `render/shadow.cpp` and `tests/test_render_shadow.cpp` say `LightingDesc`'s default sun is `(0,1,0)`. It has been `(0.4, 0.8, 0.6)` since the VQ-A fix |
| **`SR-17` clause 6** | The infinite ground is unshadowed beyond `scene.bounds`. Accepted, not closed |

## Closed

- 2026-10-02, `94deae3`: **GL lit from below** (restructure defect 4). GL shaded with `dot(n, -sun_direction)`; it now uses `+`, as every other path does (`RND-1`).
- 2026-10-02, `ab2a21e`: **the suite's reach into `../KAT`.** The 30 KAT-world agreement cases, the bookmark-drift guard, both `SPADE_KAT_*` cache variables and the two data files only they read are gone. Three `AgreementProbe` cases were added. The KAT-citing comments in render code were swept in the same merge.

## Measured

| Commit | Build tree | ctest `-L spade` (release) |
|---|---|---|
| `5501370` (`rendering/sun-convention`, before merge) | `../spade-wt/rendering/build-ninja/release` | 897 total, 864 passed, 33 skipped, 0 failed. Skips: 31 KAT-dependent cases with no `../KAT` beside a worktree, plus the standing `Fp32Exp.FullDomainSweepEveryFloatArgument` and `SlangLayouts.DeliberatelyUnboundArraysHaveNoBinding`. `gpu` label: 63 tests ran. No frame golden moved |
| `e7e1039` (`rendering/kat-reach`, before merge) | same tree | 869 total, 867 passed, 2 skipped (the two standing skips), 0 failed. 869 = 897 − 31 + 3, as predicted |

These were measured on the branches, not on the merged `master`, which also carries other realms' merges. `test-docs/07-status.md` holds the suite's baseline.

## Corrections to earlier records

- 2026-10-01: the consolidation's status said S7b was "NOT STARTED". The Vulkan raster's stages 1–2 had landed (`038b82b`, `018fc09`, `95333c5`).
