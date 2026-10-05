# GL overlays, then GL shadows: plan

**Owner:** Rendering. **Status:** approved by the lead on 2026-10-05: Q1 yes, Q2 yes, and Q3 yes, recording the device and driver with each pin. It is built after B2 step 1. Under the user's `TD-13` ruling for this box, its review notes flag the GL steps for GPU re-verification. This closes the `07-status` debt row "GL draws no shadows or overlays". Today `GlRenderer::unhonoured(RenderOptions{})` returns `{"shadows", "overlays"}`, and the sandbox HUD shows that list while GL is the path. Interface's e2e live smoke will record the window, so the gap shows on video. It needs no user decision: GL stays the editor's path (`RND-4`), graded best-effort and banded against the CPU (`RND-3`).

Two branches, overlays first. Each is red then green, gets the gcc check, and goes to review on its own.

## Part 1: overlays

**What the CPU draws** (`raster_cpu.cpp`, after field layers, when `options.overlays` is set, in shaded, wireframe and velocity modes):

| Overlay | Geometry | Colour |
|---|---|---|
| Ground grid | 21 + 21 lines, ±10 m, 1 m apart, at `scene.ground_y` (or 0) | (90, 90, 90) |
| World bounds | the 12 edges of `scene.bounds` | (90, 140, 200) |
| Spawn markers, if `options.spawn_markers` | 2 triangles per spawn, a 0.3 m diamond lifted 0.02 m | (190, 90, 170) |
| Body markers | a tetrahedron (4 triangles) per dynamic item with no mesh | (124, 147, 255) |

- Every overlay is flat and unlit, and is depth-tested and written.
- Every overlay vertex gets an inverse-depth bias of `1e-4` (`kOverlayDepthBias`), so overlays win ties with the surface they lie on.
- Lines are one pixel wide.
- This overlay grid is not the analytic ground's procedural grid (`SR-22`, `options.ground_grid`), which GL already draws.

**GL design:**
- `raster_cpu.cpp`'s overlay geometry moves into one shared function that returns world-space line and triangle lists with colours. The CPU draws from it as today, and GL uploads the same lists. One source, so the two paths cannot drift on geometry.
- One small program draws those lists, `GL_LINES` and then `GL_TRIANGLES`, with flat colour, `GL_LEQUAL`, depth writes on, and culling off. The diamonds and tetrahedra draw both sides on the CPU too.
- **The depth bias is ported exactly.** `glPolygonOffset` does not apply to `GL_LINES`. So the vertex shader moves each vertex along its view ray to the depth whose inverse is `1/d + 1e-4`. The screen position is unchanged. Inverse depth interpolates linearly in screen space on both paths, so for an overlay the near plane does not clip, the bias is the CPU's exactly. A clipped segment differs slightly, because the CPU biases after clipping.
- **Order:** after the field layers, as on the CPU.
- `unhonoured()` stops naming "overlays".

**Tests, red first:**
- `GlRendererOptions.UnhonouredNamesWhatGlDoesNotDraw` changes to expect `{"shadows"}` for `RenderOptions{}`. It is red until the implementation lands.
- New: `GpuGlRenderer.OverlaysMatchTheCpuWithinTheirBand`. A scene with a ground plane, bounds, two spawns and one meshless body, with overlays on, is drawn by the CPU and GL.
  - Every CPU overlay pixel needs a GL pixel of the same colour within 1 px, and the other way round. Line rasterization rules differ by a pixel along a line.
  - Colours are flat, so they must match exactly.
  - The miss count is pinned as a band on the 3060 Ti, with the device and the measured values recorded, like the other GL bands.
- New: `GpuGlRenderer.OverlaysLoseToNearerGeometry`. A box in front of the grid hides it, and the grid shows on the ground behind the box.
- The geometry function gets a CPU test of its own: line and triangle counts, and the end points for a known scene.

## Part 2: shadows

**What the CPU does** (`render/shadow`):
- `scene_from_world()` builds a static shadow map at load: 1024², orthographic along the sun over the scene bounds. It stores, per texel, the largest light-clip z of any static caster, with `kNoOccluder` where no caster lands.
- Each frame with dynamic bodies, it copies that map and rasterizes the dynamic casters into the copy.
- `sample_shadow()` takes a single sample, with no filtering. The texel is `clamp((p.xy·0.5 + 0.5)·size)`. A point is shadowed if `p.z < occluder − 0.05`. Outside the map's footprint it is lit (`SR-17` clause 6).
- Mesh pixels and the analytic ground's pixels both take the lookup: `combined − sun·(1 − lit)`.

**GL design:**
- **The static map is the CPU's own data.** `upload_scene()` uploads `scene.static_shadow->depth` as a 1024² `R32F` texture. Static shadows then come from bit-identical data, and GL rasterizes no static casters.
- **Dynamic casters, per frame:**
  1. Copy the static texture into a working one (`glCopyImageSubData`, core in 4.3).
  2. Draw the dynamic casters into it with `light_view_proj`, writing light-clip z with `GL_MAX` blending, which is the CPU's "keep the largest z".
  3. With no dynamic bodies, sample the static texture directly, as the CPU does.
- **The lookup is a port of `sample_shadow()`:** the same texel arithmetic with `texelFetch`, the same bias, and the same footprint rule. It runs in the mesh fragment shader and in the background shader, for the analytic ground.
- Shaded mode only, as on the CPU. `unhonoured()` stops naming "shadows". For `RenderOptions{}` it then returns nothing, and the HUD's list empties.

**Tests, red first:**
- The `unhonoured()` test expects `{}` for `RenderOptions{}`.
- New: `GpuGlRenderer.StaticShadowsMatchTheCpuWithinTheirBand`, on `shadowed_ground_with_caster`'s scene. It measures the max difference over all pixels and over interior pixels (no shadow edge within 1 px), pinned like the background band.
- New: `GpuGlRenderer.DynamicCasterShadowsMatchTheCpuWithinTheirBand`: the same scene with the caster as a dynamic body.
- New: `GpuGlRenderer.NoShadowPastTheMapFootprint`: a ground pixel outside the footprint is lit (`SR-17` clause 6).

## Steps

1. **Overlays branch:** the shared geometry function and its CPU test (no CPU pixel changes, so the frame goldens hold byte-exact), then the red GL tests, then the GL pass. Measure and pin the band. Full suite on both presets, gcc check, review.
2. **Shadows branch:** red tests, the texture upload, the dynamic pass and the lookups. Measure and pin both bands. Full suite on both presets, gcc check, review.
3. **Docs:** close the `07-status` debt row; update `01-techniques` (GL draws everything the CPU raster draws except ray-march mode) and the backlog's GL line.
4. **Interface, told at each merge:** the HUD list shrinks on its own through `unhonoured()`, and the e2e smoke will show overlays and then shadows.

## Questions for the lead

- **Q1.** Can the CPU's overlay drawing move onto a shared geometry function in Part 1? It is a refactor of `raster_cpu.cpp` that the frame goldens guard byte-exact.
- **Q2.** Should static shadows come from the CPU's uploaded map (recommended: exact data, nothing to rasterize), or should GL rasterize static casters too?
- **Q3.** Should the new bands be pinned on the 3060 Ti? The existing GL bands were pinned on the old Intel Iris Plus and still pass here.
