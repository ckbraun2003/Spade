# GL honours `RenderOptions`: plan

**Owner:** Rendering. **Status:** done. The lead approved it with Q1–Q3 answered, and it merged at `0a5e1a1` on 2026-10-03. This is item 2 of "Next for Rendering" (`../07-status.md`). It branches as `rendering/gl-render-options` once `rendering/gl-culling` (`SR-13`) has merged.

## Goal

Today `GlRenderer::draw()` ignores `RenderOptions` (`(void)options`). It draws lit meshes on whatever the caller cleared to, which in the sandbox is near-black. After this plan, the GL viewport draws what the CPU raster draws for the options the sandbox uses: sky, analytic ground, the infinite grid and the atmospheric term. It also draws wireframe. Whatever GL still does not draw is announced, never silently dropped (`L6`).

## What the CPU does: the reference to port

`render::render()` in `render/raster_cpu.cpp`:

1. **Background**, in `draw_sky_and_ground_background()`. The sky draws in every mode. The analytic ground planes, the infinite grid (when `ground_grid` is on) and the horizon term draw in shaded mode only (`SR-22`). The background **writes no depth**, so meshes draw over it.
2. **Meshes**, depth-tested.
   - **Shaded:** culls back faces (`SR-13`). The atmospheric term (`SR-17a`) applies per pixel to every material, unlit included.
   - **Wireframe:** draws edges in the unlit material colour. It never culls and has no ground or horizon term.
   - **Velocity:** lambert, with the base colour from `velocity_ramp()`. It has no ground.
3. **Overlays:** the ±10 m segment grid, the world bounds, and the spawn and body markers. Shadows come from the static map, plus dynamic casters.

`DrawMode::raymarch` is a different technique, the SDF sphere-trace, and is not a raster mode.

The Vulkan raster's `shaders/kernels/raster_background.slang` is already an fp32 port of step 1, one invocation per pixel. GL ports the same function to a fragment shader. Its per-plane data uses the same layout and the same host-side precompute: `is_front`, plus the basis from `ground_plane_basis()`.

## Scope

**In:**

1. **The background pass.** A full-screen triangle (`gl_VertexID`, empty VAO) whose fragment shader computes the sky gradient, N ground planes in an SSBO, the grid and the horizon term. The ray comes from `gl_FragCoord` with the CPU's pixel-centre convention. It is drawn first, with depth test and depth writes off, matching the CPU.
2. **The atmospheric term on mesh fragments** (`SR-17a`). This adds a world-position varying, the camera position, the sky colours, and the strength and onset. Strength is forced to 0 outside shaded mode, as the CPU does.
3. **Wireframe.** `glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)` with culling off. Edges use the unlit material or override colour, over the sky only. `GL_FILL` is restored before returning. This carries forward the requirement from `rendering/gl-culling`: wireframe draws both windings.
4. **`L6` for what remains.**
   - `raymarch` is refused with `Code::unavailable`, because it is a CPU technique.
   - Shadows, overlays and (unless Q2 says otherwise) velocity are reported, never silently dropped. Q1 decides how they are reported.

**Out:** shadows, overlays and velocity (Q2) are follow-ups. Also out:
- **Per-vertex Gouraud parity.** GL lights per pixel, which is consistent with its best-effort grade (`RND-3`).
- **The normals inverse-transpose:** the `07-status.md` debt, item 3 of the lead's round 2 list.

## Verification

All of these are `GpuGlRenderer` cases, so they are `gpu`-labelled (`TD-13`). Each is written red first, where a red state exists.

| Case | Proves |
|---|---|
| **Background colour vs the CPU.** A background-only scene (gradient sky, one plane, grid on, atmosphere on) is rendered by both paths and compared with a per-pixel, per-channel absolute difference | GL's sky, ground, grid and horizon match the reference within a band. GL is fp32 and the CPU fp64, so the band is **measured, then pinned with provenance** (`03-verification.md`), never invented. Grid-line edges are expected to dominate it. It is the first GPU colour oracle, for the background only |
| **Wireframe draws both windings.** The forward and reversed triangle give byte-identical GL frames, and both draw | `SR-13`'s wireframe clause on GL. It mirrors `RasterCpu.WireframeModeDrawsBothWindingsIdentically` |
| **No ground outside shaded mode.** A ground-only scene in wireframe gives the same GL frame as the same scene with `ground_planes` cleared | `SR-22` on GL |
| **The atmospheric pair.** One unlit far mesh, strength 0 versus strength > 0: the two frames differ, and at strength 0 the mesh pixel is exactly its base colour | `SR-17a` on mesh fragments, exact at 0 |
| **Raymarch is refused.** `draw()` returns `unavailable` and draws nothing | `L6` |
| **What GL doesn't draw is reported.** The Q1 mechanism is a pure function, so this case is not GPU-gated | `L6` |

The lead checks the sandbox visually at review: the builder and the drone box, toggling GL and CPU over the same view.

## Questions for the lead

- **Q1: how GL announces what it doesn't draw.** I recommend a pure `GlRenderer::unhonoured(const RenderOptions&)` that names the options GL will not draw. The sandbox prints the result on its HUD while GL is the path, and `draw()` keeps drawing. The alternative is for `draw()` to refuse any option it cannot honour. That is stricter, but `RenderOptions{}` defaults to `shadows = true` and `overlays = true`, so every GL caller would have to turn them off first. Either way, the sandbox side of the HUD is Interface's file, so I'll raise it with you.
- **Q2: velocity mode now or later.** It is small: a per-instance speed SSBO plus the ramp, shaded the same way, with no ground. I'd include it, because it's the only other raster mode.
- **Q3: the colour band.** Is it right to measure and pin a GL-vs-CPU background band in this branch, as the first partial GPU colour oracle? The alternative is to record the measurement only and pin it later.

## Steps

1. Cut the branch and write the cases above. All of them are red except the reporting case, which depends on Q1.
2. Write the background program and plane SSBO, the mesh-shader atmosphere, wireframe, and the `L6` handling.
3. Get a slot: `build.ps1 -Target spade_tests`, then `test.ps1 -Filter GpuGlRenderer`, then a full build and the full suite.
4. Measure the band, then pin it with provenance (Q3).
5. In `07-status.md`, update the OpenGL row and the "GL and CPU frames differ in content" debt row. Report "ready for review".
