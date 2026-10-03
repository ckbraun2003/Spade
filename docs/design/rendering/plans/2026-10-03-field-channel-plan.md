# Field channel for cameras: plan

**Owner:** Rendering, with Core for sampling and Interface for the drone heatmap. **Status:** approved; Q1–Q3 answered by the lead on 2026-10-03 (Part A now; the slipstream field after stage 3, through `PHY-3`; per-pixel targets out of scope). Part A merged at `3605ddf`. This is item 2 of "Next for Rendering" (`../07-status.md`). It answers the backlog row "Field channels for cameras" (`../../backlog.md`).

## Goal

A camera draws a registered field as a channel, on the CPU raster and on GL. The first consumer is the drone sim box. Its air-speed heatmap is sandbox code today: `sandbox/drone_view.hpp` samples the air, bins the speeds and draws each cell as an unlit quad. After this plan, the engine owns the channel, and the sandbox only chooses the field, the slice and the range.

**Done when** (the backlog's words): a camera renders a registered field as a channel on both CPU and GPU paths.

## What the design already fixes

- **A field channel is data, not appearance** (`../01-techniques-and-channels.md`). No lighting, shadow or atmospheric term (`SR-17a`) touches it.
- **The palette and its binning are part of the channel's definition**, so a CPU frame of a field is exact and testable.
- **Rendering reads only its arguments** (`L5`). Core samples the field. The renderer gets the samples as input and never calls the simulation.
- **Core samples in Publish**, from the published state, with the provider's own CPU sample function (module-API design §6 and §10). Stage 3 builds the registry and that function. Publish itself stays empty until the channel needs it.

## Requirements

- **FC-1.** A field layer MUST carry its geometry, its sample values, and its colour map. The colour map is a palette, a range and a bin count.
- **FC-2.** The CPU raster MUST colour every layer pixel exactly its bin's palette colour, whatever the lighting, shadows or atmospheric strength.
- **FC-3.** A layer MUST depth-test against meshes and MUST draw from both sides.
- **FC-4.** GL MUST draw the same layer, and MUST match the CPU within a measured band on pixels away from cell borders.
- **FC-5.** Binning MUST be total: a non-finite value, or a range of zero or less, gives bin 0 and never divides by zero.
- **FC-6.** The renderer MUST NOT sample a field itself (`L5`).

## Design

**Geometry: a slice.** A layer is a world-space rectangle: a centre, two unit axes, a width and a height in metres, and a cell count along each axis. This is the heatmap's `SliceSpec` and lattice, moved into `render`. Other geometries (a surface, a volume) wait for a consumer.

**Values: one float per cell.** A vector field reaches the layer already reduced to a scalar, for example its magnitude. The caller chooses the reduction, because it is part of what the picture means. Cell centres are the sample points.

**Colour map.** `render::FieldColourMap` holds:
- palette control points, piecewise linear (viridis first, from the sandbox);
- a range: fixed, or "auto", which is the maximum over the layer's own samples;
- a bin count. A pixel shows its bin's centre colour, as the heatmap does today.

Interface's `viridis()`, `speed_bin()` and `heatmap_color()` move into `render` unchanged, so the heatmap's exact-pixel test keeps its numbers.

**Drawing.**
- **CPU:** each cell is a flat-coloured quad, rasterized with the existing flat fill and depth test. There is no culling, no Gouraud fill and no atmospheric term. This matches today's heatmap pixels.
- **GL:** one instanced unit quad, one instance per cell, with the cell's bin in an SSBO. A separate program keeps it free of lighting and atmosphere.

**Where it lives.** `RenderScene` gains `std::vector<FieldLayer> field_layers`, drawn after the meshes. The scene, the camera and the options stay the renderer's only inputs.

**Sampling (Core, after stage 3).** Rendering needs one host call: sample a named field at N world points from the current state, into a caller-owned span. Design §6 already plans this path for `sample_medium`. Publish-side sampling, for a camera sensor stamped with its tick, comes with the camera component, not here.

## The drone heatmap needs a field that stage 3 does not register

The heatmap shows `medium(p) + Σ wake_i(p)`: wind plus every rotor's slipstream (`vehicles/rotor_wake.hpp`). Stage 3 registers `gravity`, `density` and `wind`. The slipstream is not a field. So either:

- (a) Physics registers a display-only flow field whose provider adds the wakes to `wind`. No stepping pass reads it, so no golden moves. Or:
- (b) the sandbox keeps computing the slipstream and hands the channel its own samples. The channel works the same, but the heatmap shows a field that is not registered.

I recommend (a). It is the engine model's own example: "a pressure map is a field sampled by a camera".

## Verification

| Case | Proves |
|---|---|
| Layer pixels equal their palette colour, with a lit sun, shadows and atmospheric strength 1 | FC-2, and the `SR-17a` exemption |
| A mesh in front of the layer hides it, and the layer hides a mesh behind it; both sides draw | FC-3 |
| Binning at the range ends, at zero and non-finite ranges, and on NaN | FC-5 |
| GL against the CPU on one slice: max difference over pixels whose 3x3 neighbourhood stays in one cell, measured and then pinned with its device | FC-4, as the GL background band does |
| The drone heatmap's existing exact-pixel test passes through the engine's channel | the move changed no pixel |

CPU cases are host-only. GL cases are `GpuGlRenderer` (`gpu`, `TD-13`).

## Steps

**Part A: Rendering only. It needs no Core stage, so it can start now.**
1. `render::FieldLayer` and `FieldColourMap`, with the palette and binning moved from the sandbox. Write the tests first.
2. CPU raster drawing, then GL drawing. Then measure the GL band and pin it.
3. Interface switches the heatmap to `field_layers`, still with sandbox-computed samples (option (b) for now).

**Part B: after Core's stage 3.**
4. The sandbox samples through Core's host call, using `wind`, or the flow field if (a) is chosen.
5. In `07-status.md`, mark the field channel built and close the backlog row.

## Questions for the lead

- **Q1.** May Part A start before Core's stage 3? It takes samples as input, so it does not depend on the registry.
- **Q2.** The heatmap's air field: (a) a display-only flow field, owned by Physics, or (b) keep the sandbox's own sum? This needs Physics' view.
- **Q3.** A per-pixel float target (a true data channel, as a sensor would read) is out of scope here. The samples themselves are the data until camera sensors exist. Agreed?
