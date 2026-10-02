# Rendering library

Techniques, channels, cameras, the scene representation, the CPU, OpenGL and Vulkan backends, and render goldens and bands. Code: `engine/render/`, `engine/render_gl/`, `engine/render/vulkan/`, the raster kernels in `engine/shaders/kernels/`, and the `tests/test_render_*.cpp` suites.

## Read in this order

1. `01-techniques-and-channels.md`: what a camera is, which techniques and backends exist, their grades.
2. `02-scene-and-appearance.md`: how a world becomes a render scene, and what materials, light, ground and atmosphere mean.
3. `03-verification.md`: goldens, the agreement test, GPU bands, how to write a render test.
4. `07-status.md`: what exists today and what is open.
5. `00-decisions.md`: the ruling behind an ID, and whether it is in force.

## If you're here to…

| …do this | read |
|---|---|
| change how a pixel is lit | `02-scene-and-appearance.md` (materials and light), then the shared functions in `render/scene.hpp` |
| add or resume a backend (the Vulkan raster, say) | `01-techniques-and-channels.md`, then `03-verification.md` (GPU paths) |
| add a channel or a camera sensor | `01-techniques-and-channels.md` (channels, camera) |
| touch or regenerate a golden | `03-verification.md` (goldens) |
| measure agreement bands on a world | `03-verification.md` (agreement) |
| find out whether `RS-n`, `SR-n` or `PA-n` is in force | `00-decisions.md` |

## Series

- **Owned here:** `RS1`–`RS15`, `SR-*`, `PA-*` (the S7a render program's plan amendments), `A8` (the camera lane), and new rulings `RND-n`.
- **Shared:** `RS5` with Core. Core owns the world-file format; Rendering owns what its appearance fields mean.
- **Quoted, not owned:** the laws `L1`–`L8` (`../00-charter.md`), and `SL9c` (the velocity draw mode) and `SL10` (the sandbox's same-path rule), whose homes the `SL` split assigns (`../README.md`).

Cite as `RS4`, `SR-17a` or `RND-1`, by document and section, never by line number. Status lives in `00-decisions.md` and `07-status.md` only.

## History

The source text for everything here is `../superseded/2026-09-consolidation/03-world-and-render.md`, with the render rows of `07-status.md` and `08-lessons.md` beside it. KAT's originals are in `../superseded/kat-originals/`. Active plans go in `plans/`, dated.
