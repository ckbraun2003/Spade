# Rendering — techniques, channels and cameras

**Owner:** Rendering. **Normative.** Target design in the engine model's terms (`../01-engine-model.md`); where the code is today is in `07-status.md`.

## The model

A **camera** is a **technique** plus one or more **channels**. The same camera serves as a viewport or as a sensor. It reads only the published frame state, so rendering can never change stepping (`L5`).

- **Technique:** how rays meet the scene. Raster, ray-march, ray-trace.
- **Backend:** where the technique runs. CPU, OpenGL, Vulkan. A technique can have several backends.
- **Channel:** what each pixel carries. Colour, depth, object ID, or any registered field (a pressure map is a field sampled by a camera).
- **Grade (`L3`, `L4`):** declared per technique and backend. The CPU path is the reference. A GPU path is banded against it and is never a golden source.

## Techniques and backends

| Technique | Backend | Role | Grade (proposed, `RND-3`) |
|---|---|---|---|
| Raster | CPU (`render/raster_cpu`) | The reference picture, the headless and CI surface, and the no-GPU fallback | **reference**. It computes in fp64, which the charter's numerics note allows when declared, and its frames are byte-exact goldens |
| Ray-march | CPU (`render/raymarch`) | Sphere-traces the SDF that collision evaluates. It is the geometric truth the raster is checked against (`RS4`), not a presentation path | **reference** for coverage only |
| Raster | OpenGL (`render_gl/GlRenderer`) | The interactive viewport when a GL 4.3 context exists | **best-effort** until it has a band |
| Raster | Vulkan compute (`render/vulkan/`) | Fast, and a future camera sensor without a window | **banded**: coverage against the CPU (`compare_silhouettes`) plus a colour oracle, which does not exist yet |
| Ray-trace | none | Not designed yet | — |

**Rules for every technique:**
- **A pure function of its inputs.** No clock, no RNG, no static mutable state. The same scene, camera and options give the same pixels on that backend.
- **One shading model.** `shade_vertex_color()` and `sky_gradient_color()` in `render/scene.hpp` are the one definition of a lit pixel. A backend ports them; it does not reinvent them. One sun convention on every path (`RND-1`).
- **A partial technique carries a partial name** (`RND-2`). A backend that can only draw part of the frame exposes an entry point named for that part, as the Vulkan raster's `render_background()` does, until it can draw everything a full `render()` promises.
- **Fallback is announced** (`L6`). A caller that wanted a GPU backend and got the CPU says which path drew the frame, durably and visibly.
- **The target is caller-owned** (`PA-1`). A technique writes into memory it is handed and allocates no frame storage of its own.

## Channels

| Channel | Today |
|---|---|
| Colour, BGRX8 | Built on every backend |
| Depth | Each backend has an internal depth buffer, but none exports it |
| Object ID | None |
| Field | None. The drone sim box draws its air-velocity heatmap from sandbox-side quads coloured by unlit palette materials, which shows the need (`../backlog.md`, "Field channels for cameras") |
| Velocity | `DrawMode::velocity` is a debug colour encoding of body velocity, not a channel |

A field channel samples a field the camera requests from the published frame state, then maps it to colour through a declared palette and range. The palette and its binning are part of the channel's definition, so a CPU frame of a field is exact and testable, as the drone heatmap's exact-pixel test already is.

## Camera

Today a camera is the plain value `render::Camera` (position, orientation, vertical FOV, near and far planes), passed with every call. It is not an object, and nothing in the engine stamps or schedules a rendered frame.

Target: a camera is an object with a camera component (technique, channels, resolution, cadence). As a **sensor**, it renders at its cadence from the frame state of the tick it is stamped with, into a slot from a frame pool. As a **viewport**, the editor owns its cadence. The engine does not own a frame pool yet. The only shipped pools are KAT's (`../consumers.md`).

## What the code has to become

1. Techniques registered as modules, with a declared grade per backend, chosen per camera.
2. Channels beyond colour, starting with the field channel.
3. A camera component and a camera sensor, fed from Publish.
4. The Vulkan raster resumed as a technique (`../backlog.md`, "GPU rasterizer as a render technique"), with colour guarded before it is selectable.
