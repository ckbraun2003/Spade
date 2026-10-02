# Rendering — status

**The only place that says what exists today.** Checked against the tree at `master` `0dd8bdc` on 2026-10-01, by reading the code and CMake. No build tree existed in this checkout, so nothing below is a measured test result. Branches not yet merged are named where they change a row.

## Specified vs built

| Specified | Built | Where |
|---|---|---|
| CPU raster technique: shaded, wireframe, ray-march and velocity modes, materials, sun, ambient, sky, shadows, overlays | **Built** (S7a, `9778260`, 2026-09-02) | `engine/render/` → `spade::render`, installed |
| Scene representation (`RS2`, `RS3`): tessellation, CSG meshing, glTF, analytic ground (`SR-17`) | **Built** | `render/scene.*`, `tessellate.*`, `csg_mesh.*`, `gltf.*` |
| Atmospheric term (`SR-17a`) | **Built**, default strength 0. The sandbox turns it on | `RenderOptions::horizon_blend_strength` |
| Analytic infinite grid | **Built**, default off, shaded mode only. The sandbox turns it on | `RenderOptions::ground_grid` |
| Agreement machinery (`RS4`, `SR-30`) | **Built** | `render/agreement.*` |
| Agreement bands | **None after `rendering/kat-reach` merges.** The only bands were measured on KAT's worlds, read from `../KAT` | `../backlog.md` |
| OpenGL raster backend | **Built, partial.** Instanced meshes with lambert/unlit shading only. It ignores `RenderOptions`: no sky, analytic ground, grid, shadows, overlays, draw modes or atmospheric term (the sandbox clears to a flat colour). Optional module behind `SPADE_RENDER_GL` | `engine/render_gl/` → `spade::render_gl` |
| Vulkan raster technique | **Stages 1–2 written, in no build graph.** Background only (sky, analytic ground, grid, horizon). The kernel `raster_background.slang` is not compiled, and no test calls `GpuRasterizer`. **Paused** by the restructure, to return as a technique | `render/raster_gpu.hpp`, `render/vulkan/`, `shaders/kernels/raster_background.slang` |
| Channels: depth, IDs, fields | **Not built.** Colour only | — |
| Camera component, camera sensor, frame pool | **Not built.** `render::Camera` is a per-call value | — |
| Ray-trace technique | **Not designed** | — |
| Frame, tessellation and CSG goldens (`RS13`) | **Built**: 5 frames, 7 primitives, 2 CSG subtrees | `tests/golden/render/` |
| Render bench | **Built**, recorded not gated | `tests/bench/bench_render.cpp` |
| Render-cadence invariance test (`RS13`) | **Not in this repository** (it was a KAT-side case) | — |

## Open items — needs a user decision

1. **Sign `RND-1`** (the sun convention). It is in force in the code once `rendering/sun-convention` merges.
2. **The grade of each technique and backend** (`RND-3`, proposed in `01-techniques-and-channels.md`).
3. **When the Vulkan raster resumes as a technique**, and whether the OpenGL backend stays as a technique of its own or retires once Vulkan can draw a full frame.
4. **The orphaned `SR-nn` rows** in `00-decisions.md` are reconstructed from their code citations and are unsigned. They are the ruling as the code states it, not as anyone signed it.

## Open items — debt

| Item | Detail |
|---|---|
| **GL lit from below** (restructure defect 4) | GL shaded with `dot(n, -sun_direction)` and the CPU with `+`. **Fixed on `rendering/sun-convention`** (`5501370`), approved, awaiting build slot #3 |
| **`render::Lighting{}`'s default sun is below the horizon** | `{-0.35, -0.86, -0.37}` points away from the sun under `RND-1`. Four hand-built frame goldens shade under it: `box_shaded_with_overlays`, `cylinder_static_dynamic_box_top_down`, `shadowed_ground_with_caster` and its `_atmospheric` pair. Flipping it is **one deliberate regeneration, measured on both Windows and Linux** before promotion. World-loaded scenes are unaffected, because `LightingDesc`'s default `(0.4, 0.8, 0.6)` is correct |
| **Normals under non-uniform scale** | Both paths transform normals by `mat3(model)` with no inverse-transpose (`transform_normal()` on the CPU, the GL vertex shader), so lambert shading is wrong on any non-uniformly scaled item. **No golden uses non-uniform scale:** frame goldens are rigid, and props and SDF poses take uniform scale only. So the fix moves no golden. It does affect the sandbox builder today, whose objects take a per-axis scale |
| **Installed headers that cannot compile** (found by Core) | `render/vulkan/background_pass.hpp` includes `compute/vulkan/context.hpp`, and through it volk. `install(DIRECTORY render/ ...)` installs it in every configuration, so a consumer that includes it fails to compile. It is latent while nothing includes it. When the Vulkan raster returns as a technique, either exclude `render/vulkan/` from the install or keep that header Vulkan-free |
| **GPU colour unguarded** | The Vulkan raster's only evidence is coverage. A colour oracle is required before it is selectable (`03-verification.md`) |
| **No agreement bands on Spade's own worlds** | `../backlog.md` |
| **No render-cadence invariance test** | It holds by construction. A Spade-side test is owed (`RS13`) |
| **GL and CPU frames differ in content** | Because GL ignores `RenderOptions`, a scene looks different on the two paths beyond numerics: no sky, ground or shadows on GL |
| **Stale comments** | `render/shadow.cpp` and `tests/test_render_shadow.cpp` say `LightingDesc`'s default sun is `(0,1,0)`. It has been `(0.4, 0.8, 0.6)` since the VQ-A fix |
| **KAT-citing comments in render code** | About 20 lines in `engine/render/`, `raster_background.slang` and `tests/test_render_*.cpp`. **Swept on `rendering/kat-reach`** (`e7e1039`, comment-only), awaiting build and merge |
| **`SR-17` clause 6** | The infinite ground is unshadowed beyond `scene.bounds`. Accepted, not closed |

## Corrections to earlier records

- 2026-10-01: the consolidation's status said S7b was "NOT STARTED". The Vulkan raster's stages 1–2 had landed (`038b82b`, `018fc09`, `95333c5`).
