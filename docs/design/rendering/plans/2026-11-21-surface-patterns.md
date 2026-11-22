# Procedural surface patterns in the rasters (O168, Cameron's choice "Patterns, after pressure fixes") -- design, for the lead's review before code

Rendering, 2026-11-21. Nothing is built. This is the lead's option A of the texture estimate: asphalt grain, kerb stripes, grass and a checker, as functions of position, in the GL and the CPU raster, built once as an engine module the camera sensor can share later. Image textures stay out (option D is a later choice, once Cameron has seen these).

## The ruling it needs, and the record

A dated amendment to `RS6` ("no textures") and `RS15` (textures out of scope), in `00-decisions.md`, on Cameron's O168: procedural patterns are allowed in the viewport and both rasters; image textures are not. `RND-13` (the camera sensor may texture; procedural first) already covers the camera. The one-line record goes in with the module's first commit.

## The module: `engine/render/surface_pattern.hpp` and `.cpp`

No libm, no clock, no allocation in the evaluation (SR-14): `+ - * /`, `floor` on floats that are exact, and 32-bit integer maths only, so the same inputs give the same bits on the CPU and in GLSL.

```
enum class PatternKind : uint8_t { none, checker, stripes, asphalt, grass };
struct SurfacePattern {
    PatternKind kind = none;
    glm::vec3 colour_a, colour_b;   // linear; the pattern mixes between them
    float scale_m = 1.0f;           // the feature size: a checker square, a stripe's width, a noise cell
    float contrast = 1.0f;          // 0..1: how far the mix goes from colour_a toward colour_b (noise kinds)
    uint32_t seed = 0;              // the pattern's own hash seed: the same seed, the same surface
    uint8_t axis = 0;               // stripes only: 0 x, 1 y, 2 z, the axis the stripes run ALONG
};
// The albedo of a surface point: `position` is the point in the pattern's own space (see "Space"), `footprint_m` the world size of one pixel there (0: no filtering).
[[nodiscard]] glm::vec3 pattern_albedo(const SurfacePattern& p, const glm::vec3& position, float footprint_m);
```

- **checker**: the parity of `floor(x/s) + floor(y/s) + floor(z/s)` (the ground and a wall's plane both read right in 3 D), `colour_a` or `colour_b`.
- **stripes**: the parity of `floor(c/s)`, c the coordinate across `axis`; kerb red and white is one stripes pattern.
- **asphalt**: value noise at three octaves (cell `s`, `s/2`, `s/4`) on an integer lattice, plus a speckle of 1 mm grains: all hashed from `seed` and the lattice cell, `colour_a` the dark base, `colour_b` the light aggregate, `contrast` the mix depth.
- **grass**: the same noise, stretched 4:1 along one direction at the finest octave, on greens.
- **Hash and noise**: one 32-bit integer hash (PCG-style, no multiplication overflow dependence beyond unsigned wrap), the lattice corners' values in fixed point, interpolated with an integer smoothstep. A float enters only as `floor(position/s)` and a fraction in `[0, 1)`; both are the same operations on both sides.
- **Filtering (the aliasing a plane to the horizon would show)**: when a feature is smaller than the pixel footprint, the pattern blends toward its own mean colour, linearly from `feature = footprint` (full pattern) to `feature = footprint / 2` (the mean), per octave. That is the analytic version of a mip chain, needs no storage, and is a function of the footprint alone: the rasters compute the footprint from depth and the projection and hand it in.

## Space (what "position" is)

The caller picks it, by one rule: **statics and the ground use the world position** (a track is fixed); **a body that moves or turns uses its object-space position** (the pattern stays on it). The CPU raster knows both at every pixel (it interpolates the object-space vertex position it already has), GL passes the object-space position as a varying alongside the world one; an SDF hit has its local point from the march.

## Where it is used

`Material` gains `std::optional<SurfacePattern> pattern`. In the lit-pixel function, a patterned lambert material takes `pattern_albedo(...)` in place of `base_color` before the light (unlit and emissive are left as they are). The analytic ground takes the ground material's pattern at world (x, z). The camera sensor's shading calls the same function with its ray hit when it needs it (its own pool of image-like textures, 6c, is a separate thing and later).

## Core's part

The world file's material block gains a strict `pattern: {kind, colour_a, colour_b, scale_m, contrast, seed, axis}` (unknown key refused by its path, `scale_m` > 0, `contrast` in [0, 1], colours in [0, 1], `axis` only for stripes); Core's schema and reader, about half a day, and a world without it is unchanged to the byte (no digest moves). The world-file spelling of the colours, and whether the material's `base_color` doubles as `colour_a`, I leave to Core.

## The band, and the tests (RED first)

1. **The module on the CPU**: golden values at fixed points for each kind (frozen, with the seed), the hash's known outputs, periodicity of the checker and the stripes, the same seed gives the same surface and another seed another, the footprint blend is monotone and ends at the mean colour.
2. **Bit-equal integer parts**: the lattice hash and the integer smoothstep are evaluated by a GL compute pass over a dense grid of cells and compared to the CPU as integers: **equal, no tolerance**.
3. **The colour band**: the full `pattern_albedo` over a dense grid of 1 M positions and 5 footprints through GLSL and the CPU: the 8-bit encoded result within 1 level, and the share of any 1-level difference reported (the same style as the rasters' existing band, and held to it).
4. **In the rasters**: a plane with each pattern drawn by the CPU raster and the GL raster at one camera: the frame difference within the rasters' existing band; the checker's squares land where `floor(x/s)` says at known pixels; a moving body's pattern turns with it (the object-space rule); the ground takes the ground's pattern.
5. **The world-file reader's refusals** (Core's tests).

## Cost, and order

About 4 days of mine: the module and its CPU tests (1), the CPU raster integration for meshes, SDF hits and the ground (1), the GL shader port with the grid and the integer-equality tests (1.5), the scene band test, the viewport and the decisions amendment (0.5). Core's 0.5 day on the reader sits alongside. It starts after the O158 pressure-film code and the camera tool, as ordered.

## Settled with the lead (2026-11-21)

1. **The band's unit:** the 8-bit sRGB output level, at most 1 level in 255 per channel, counted (the same unit as RND-13's camera filter), plus the integer hash and the noise lattice **bit-equal**.
2. **Colours:** a pattern carries its own two colours. The material's `base_color` is the fallback (used when patterns are off) and the far-field mean colour of the footprint blend where that is simpler; it is never `colour_a` by default.
3. **SDF boxes:** yes, if it is the same function at the same shading point and takes under about half a day (the SDF hit already has its object-space point); CSG gets it only if that is free. Otherwise meshes and the ground come first, and SDF is named as the next step.

Ordering as Cameron set it: after the O158 pressure code and the camera tool. This doc stays on its branch until it goes in with the module's first commit.
