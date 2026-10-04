# Ray-marched CSG in the raster (B2): plan

**Owner:** Rendering. **Status:** draft, for the lead's review. The user ruled on 2026-10-04 to plan B2 now (`../../backlog.md`). B2 replaces `RS3`'s "CSG meshed at load", so the `RS3` replacement below goes to the user for signature before any code. B1 (a world-space cell size, `rendering/csg-cell-size`) lands first regardless.

## What B2 is

The CPU raster and GL stop drawing CSG subtrees from a surface-nets mesh. Each frame they ray-march each CSG subtree over the pixels its bounds cover, and write colour and depth into the same buffers the meshes use. This is the same sphere trace the reference ray-march runs (`render/raymarch`, `SR-28`), restricted to one subtree and depth-tested against everything else. Tessellated unions and glTF stay meshes.

It fixes defect B at the root. A sphere tracer never steps past a surface, so a wall of any thickness draws, and nothing folds. The CSG part of `RS4` then holds to the reference's own epsilon.

## Cost, measured

`BM_CsgFrame` (`tests/bench/bench_render.cpp`, branch `rendering/b2-measure` at `f1b896c`) draws one frame of `shower`'s shell, an 8 m tapered bowl, three ways. Release MSVC, single-threaded (both paths are), the 16-core box, median of 3 repetitions, real time in ms per frame. Runs vary by up to 20%.

| View (shell coverage) | Size | Background only | Old 48-cell mesh | B1 mesh (0.05 m, cap 160) | Full-frame ray-march |
|---|---|---|---|---|---|
| Outside (14–17%) | 640x360 | 17.9 | 31.4 | 171 | 126 |
| Inside the bowl (100%) | 640x360 | 17.9 | 39.8 | 131 | 92.8 |
| Outside (15–17%) | 1504x1003 | 108 | 141 | 281 | 764 |
| Inside the bowl (98–100%) | 1504x1003 | 108 | 263 | 314 | 596 |

What it shows:
- **The CPU raster's cost of a mesh is mostly per triangle.** B1's mesh of this shell has about 320 000 triangles. Over the background, it adds 113–153 ms a frame at 640x360 and 173–206 ms at 1504x1003, about 0.35–0.65 µs per triangle. The old 48-cell mesh (27 000 triangles) added 14–22 ms at 640x360. At 1504x1003 it added 33 ms outside and 155 ms inside, where filling the whole frame dominates. **So B1, as on `rendering/csg-cell-size`, makes a frame with a large CSG subtree 1.2–5.4x dearer on the CPU raster**, most at low resolution. The B1 bench measured load time only, so this is new.
- **A ray-march's cost is per pixel.** Every pixel hits in the inside view, and the march costs 0.40 µs per pixel at both sizes. The outside view marches every sky ray to the 1000 m far plane, which costs 0.51–0.55 µs per pixel.
- **B2 marches only the subtree's screen rectangle, and clips each ray to the subtree's bounds.** So a B2 frame costs about the frame without the subtree, plus the rectangle's pixels at about 0.4–0.5 µs each. These are estimates from the table, not measurements of B2. The outside rows assume the rectangle is about twice the shell's coverage:

| View | Size | B1 mesh, measured | B2, estimated |
|---|---|---|---|
| Outside (rectangle about 30% of the frame) | 640x360 | 171 | about 55 |
| Inside | 640x360 | 131 | about 110 |
| Outside | 1504x1003 | 281 | about 340 |
| Inside | 1504x1003 | 314 | about 700 |

- **Where each wins on the CPU.** A mesh costs about 0.35–0.65 µs per triangle, and B2 about 0.4–0.5 µs per marched pixel. So, roughly, B2 is cheaper when the subtree's screen rectangle has fewer pixels than its mesh has triangles. For this 8 m shell (320 000 triangles), B2 always wins at 640x360, and at 1504x1003 it wins while the rectangle stays under about 20% of the frame. A gate-sized 2 m subtree gets the 48-cell floor, about 30 000 triangles (`gate_square`'s shape has 30 400 at 48 cells). So B2 loses once it fills more than about 2% of a 1504x1003 frame.
- **The step cost grows with the subtree.** Each step evaluates the whole subtree. The shell has 3 nodes and `maximal`'s CSG subtree has 9, so expect a step to cost roughly 3x more there.
- **GL is unmeasured.** On a GPU, 330 000 triangles cost well under a millisecond, and so does a fragment ray-march of a few hundred thousand pixels at about 40 steps. The 3060 Ti and the old Intel Iris Plus will differ by about 10x. Step 3 below measures it before GL ships.

**Who pays.** KAT's host draws its frames with the CPU raster (`spade::render::render`, `kathost_render`), at sizes like the table's. Today only Spade's test worlds (`shower`, `maximal`) have CSG subtrees: the drone scenes draw gates from glTF. So neither B1's per-frame cost nor B2's lands on a real scene yet. For camera sensors, `A6` already prices per pixel, and B2 fits that model better than B1.

## Design

**CPU raster (`render/raster_cpu`):**
- `scene_from_world()` keeps, for each CSG root, the subtree's own program (`csg_mesh.cpp`'s `extract_subtree()`, made public), its world bounds, its material and its draw-item index. `RenderScene` gains a `csg_subtrees` list.
- A new pass, after the meshes and before field layers, does this for each subtree:
  1. Project the bounds' 8 corners to a pixel rectangle. If the bounds cross the near plane, use the whole frame.
  2. For each pixel in the rectangle, generate the ray exactly as `raymarch.cpp` does. That code moves into a helper both paths call, so the rays are bit-identical.
  3. Clip the ray to the bounds, and to the near and far planes in camera-space depth (the reference's frustum rule).
  4. Sphere-trace the subtree with `kRaymarchSurfaceEpsilon` and `kRaymarchMaxSteps`. A ray that starts inside the solid marches through it, as the reference does, which is how the raster's back-face cull reads (`SR-13`).
  5. On a hit, depth-test against the raster's inverse-depth buffer. Shade with `shade_vertex_color()`, using the SDF gradient as the normal, as the reference does. Add the shadow-map lookup at the hit point and the `SR-17a` horizon blend.
- **Material:** the subtree's root material, as the mesh uses today (`scene.cpp`: a CSG mesh is one implicit submesh, `SR-11`). The reference resolves per leaf. Per-leaf colour would be a visible change, so it is a separate question (Q4).
- **Wireframe** (`SR-13`) has no surface to outline in a ray-march, so it keeps drawing the mesh.
- **Shadows:** the static shadow map keeps rasterizing the mesh as a caster. Ray-marching the shadow map would cost seconds at load.
- **Velocity mode:** CSG is static, so its pixels write zero speed, as static meshes do.
- **Purity:** a pure per-pixel function with no threads, so frame goldens stay byte-exact (`TD-1`).

**GL (`render_gl/GlRenderer`):**
- Each subtree's program (nodes, transforms, parameters) goes into a shader storage buffer. SSBOs are core in GL 4.3, the version GL already requires.
- Draw the subtree's bounding box as a proxy, with front faces, or back faces when the camera is inside the box. The fragment shader ray-marches from the box entry, writes `gl_FragDepth` in the mesh pass's depth convention, and discards a miss. The proxy gives the screen rectangle for free.
- **The SDF evaluator in GLSL is a hand port**, as the background pass is (`kBackgroundFragmentSrc` ports `raster_background.slang`). It follows `sdf_eval.slang`'s op-order notes. GL is graded best-effort and banded against the CPU (`RND-3`), so it needs no bit-parity. Generating GLSL from `sdf_eval.slang` with `slangc` would avoid a third copy. But that module binds to Vulkan layouts, and `slangc`'s GLSL output for 4.3 core is untested here, so it is a spike option, not the plan.
- `GlRenderer::unhonoured()` gains nothing: GL draws CSG either way.

**Vulkan raster** (`RND-4`, later): `sdf_eval.slang` already exists, so the same pass is a compute kernel when the Vulkan raster returns.

## What B2 changes

- **`RS3`, proposed new wording for the user's signature:** "Three geometry paths: tessellated unions, CSG subtrees ray-marched per pixel within their bounds, and glTF, plus ray-march truth. The CSG mesh is kept, at fixed resolution, for wireframe and shadow casting only."
- **Agreement bands:**
  - The CSG cases' d falls to near 0: `maximal.csg_against_sky` is 0.0011 now, against a band of 0.002, and `shower.outside_from_below` is 0.0028, against 0.005.
  - It will not be exactly 0. The raster tests depth against meshes, the reference marches the whole scene, and other primitives are still tessellated.
  - The bands get a version 2 (`SR-31`), re-measured on msvc-release, msvc-debug and gcc-release, with Test/Docs' margin (`TD-2`).
  - `shower`'s inside view, dropped for defect B, can return as a case.
- **B3's two-cell rule and the fold warning:** obsolete for shaded frames. A thin wall draws exactly. The mesh still feeds wireframe and shadows, so the warning narrows to say so: "draws with holes in wireframe and casts a leaky shadow". The rule stops being an authoring requirement.
- **B1's fine mesh** is then needed only for wireframe and shadows. Q2 asks whether those return to a coarser mesh to win back the per-frame cost.
- **Goldens:**
  - The CSG mesh goldens stay, since the mesh is still built.
  - No current frame golden has CSG: the five are hand-built scenes.
  - A new CSG frame golden would pin B2's pixels, cross-checked by the Docker leg (`TD-12`).
- **`RND-6` and A1:** unaffected. A1 is the same machinery for a heightfield background, and it could share this pass's marcher later.

## Steps, once the plan and the `RS3` replacement are signed

Each step goes red then green, gets the gcc check, and goes to review.
1. **CPU pass, tests first.** Red:
   - a 0.02 m wall, under one B1 cell, draws without holes from inside: d against the reference under 0.001;
   - a mesh primitive half inside a CSG subtree occludes correctly both ways;
   - a camera inside the CSG solid sees what is behind it;
   - the same frame twice is byte-identical.
   Green: the pass. The bench gains a B2 row, which replaces the estimates above with measurements.
2. **A CSG frame golden** (`TD-1`), its hash pending until the Docker leg reproduces it (`TD-12`).
3. **GL pass.** Measure its cost on the 3060 Ti. Then a `GpuGlRenderer` band test, GL against the CPU on a CSG scene, pinned on one device.
4. **Agreement bands, version 2.** Re-measure on three toolchains, return `shower`'s inside view, and get Test/Docs' margins.
5. **Narrow the fold warning** to wireframe and shadows, and settle Q2.
6. **Docs:** `RS3` row, `01-techniques` (CSG drawn by ray-march in the raster), `07-status` (defect B closed).

## Questions for the lead

- **Q1.** Is the cost trade acceptable for the CPU raster? B2 is cheaper than B1 when a CSG subtree is small on screen, and dearer when it fills a large high-resolution frame (about 700 ms against 314 ms inside the bowl at 1504x1003). Real scenes have no CSG today.
- **Q2.** Should wireframe and shadow casting go back to the old 48-cell mesh once B2 draws shaded frames? That wins back B1's per-frame cost in those modes (up to 5.4x on a large subtree) at the price of coarser wireframe and shadow edges.
- **Q3.** GL: a hand-ported GLSL evaluator (recommended), or a `slangc` spike first?
- **Q4.** Material: keep one material per CSG subtree (recommended, no visible change), or resolve per leaf like the reference?
- **Q5.** B1's per-frame cost is real today, before B2. Should `07-status` record it as debt now, with B2 or Q2 as its fix?
