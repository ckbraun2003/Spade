# Geo terrain: a textured, render-only map (SP-4, KAT's geo plan G2) — design

Status: **draft for review**, 2026-11-17. Nothing is built. Source: KAT's `2026-11-17-geo-locations.md` (owner decisions D154–D159, REQ-G07, G10, G14, G16); the lead's L85 assignment. Rendering owns the mesh and the drawing. Interface owns the fetch and the cache (SP-5).

The first target is the Bonneville Salt Flats: an area under 1 km, NAIP imagery at about 0.6 m per pixel (about 1 670 pixels a side), a 3DEP height grid in float32. The terrain is **for the eye only**. It adds no collision, and nothing in `spade_sim` reads it.

## What the engine takes

Rendering never reads files, resolves URLs or does geodesy. The caller (the cache reader, SP-5's side) hands it decoded data, already in metres from the anchor:

```
TerrainInput {
  heights      float32 grid, nx x ny, rows north to south, columns west to east, METRES UP FROM THE ANCHOR (y of world space)
  image        RGB8, w x h, rows north to south
  extent       east_min, east_max, north_min, north_max: metres from the anchor, the OUTER edges of both rasters
}
attach_terrain(RenderScene&, const TerrainInput&, const TerrainSpec&) -> Result<TerrainStats>
```

`TerrainSpec` carries the cell cap and the lit/unlit choice below. `TerrainStats` reports the cells, the triangles and the largest vertical gap between the mesh and the source grid, so a decimation is never silent.

- **Refused by name**, scene left unchanged: a side more than one source cell over 1 000 m (REQ-G07), a non-finite or "no data" height, an extent that is empty or reversed, a grid or image under 2 x 2, rasters of different size ratios than the extent allows, an image over the GL texture limit (a caller can ask for `GlRenderer::supports()` first). The caller shows the reason (REQ-G15); the scene then draws plain ground.
- **No terrain attached** is today's scene, bit for bit. A missing map is the same as no call (REQ-G14).

## The mesh

- **Placement.** World x = east, world z = −north, world y = height from the anchor (x east, y up, z south; REQ-G05). The positions are already in world metres, so the draw item's `local_to_world` is the identity. The anchor (0, 0, 0) lies inside the extent or not; either is fine.
- **Heights are not corrected for the earth's curve.** KAT's plan says the flat plane is about 8 cm off at 1 km. The terrain stays on the flat plane, so it agrees with the physics ground (the world's own plane at the anchor) wherever the land is flat, and the drone does not appear to hover over the map. The 8 cm is the GNSS's and is recorded in KAT's plan.
- **Resolution.** A regular grid of `n x n` cells over the extent, `n = min(source cells, max_cells)`, default `max_cells` 128. At 1 km that is 7.8 m a cell and 32 768 triangles, about 20 ms on the CPU raster at its measured 0.35–0.65 µs a triangle. A 10 m 3DEP grid (100 cells) is used one to one. The aerial image carries the detail, not the mesh.
  - Each vertex is the source grid sampled bilinearly at its position, clamped at the edges. Normals come from central differences of the vertex heights, so the surface shades smooth. All in float with fixed evaluation order, so every platform builds the same bytes.
  - `TerrainStats` reports the largest gap between the mesh and the source at the cell centres. If a real site needs more than a regular grid, an error-driven decimation is a later task (YAGNI); the stat tells us when.
- **One draw item**, in its own label ("terrain"), in `statics`. It does not cast shadows (a 1 km map would waste the sun's shadow-map resolution, and a flat map shades nothing onto the drone); it **receives** the drone's shadow, as `see_through` items do. Sensor views draw it as a surface and segmentation colours it "terrain".
- **The terrain wins over the render ground.** The world's ground plane is also drawn (a bounded mesh plus the analytic plane beyond it), and on the salt flats the map's heights are within about 1 m of it, so the two would fight. `attach_terrain` lowers the render copy of each ground plane (the items and `GroundPlane` records that share a plane node's label) to just under the map's lowest vertex. Only drawing moves: `ground_y`, the physics ground and the SDF are untouched (REQ-G16). Beyond the map's edge the plane shows a step of at most that gap, which is invisible from a drone's height. *This is the one choice I most want a second opinion on*; the other is to clip the ground items inside the extent, which means splitting triangles.

## Texturing

Today's glTF path has no textures and the terrain does not use it: the mesh is built in memory. The aerial image is **planar**: it is the map seen from straight above, so a pixel's colour is a function of its world (x, z) alone. That needs **no UV coordinates** and no change to `MeshData`:

```
u = (x - x_west) / (x_east - x_west)      v = (z - z_north) / (z_south - z_north)      (z = -north, row 0 is the north edge)
texel = bilinear(image, u, v), clamped at the edges, colour = texel / 255
```

- **CPU raster draws it, GL draws it. Neither refuses.** The CPU is the golden source and draws the website's stills, so a map it could not draw would leave the stills without ground. The CPU already recovers a perspective-correct world position per pixel for shadows and the atmosphere (SR-24); the texture takes the same position. The terrain's material is white, so the vertex shading is the light alone, and the texel multiplies it per pixel after the shadow and before the atmosphere. Because lambert is linear in the base colour, a constant image must equal the plain-material draw (the test below).
- **GL:** a 2D texture (RGB8, linear filter, clamp), the world position it already passes to the fragment shader, the same formula in GLSL. A texture over `GL_MAX_TEXTURE_SIZE` is refused by name in `supports()`. GL lights per pixel and the CPU per vertex (SR-18), so the two differ by design; the pinned band is measured, as the field layer's was.
- **Colour:** the texel is used as a base colour like any material's, with no sRGB decode, as every colour in the engine is today. `TerrainSpec::lit` picks the shading: *lambert* (default; receives shadows, the aerial photo's own light and the sun add, so it can look bright) or *unlit* (the photograph as it is, no shadow from the drone). I recommend lambert, to keep the drone's shadow, and tuning the sun on the first real frame.
- **Other draw modes.** Wireframe draws the mesh edges, no texture. Depth, normals and segmentation draw the geometry. Velocity gives it speed 0. The exact ray-march mode draws the world's SDF and does not draw the terrain, as it does not draw field layers, and the agreement matrix keeps it out of frame as it keeps the heightfield (RND-6). Said in `07-status.md`, not silent.

## The image decoder

**Recommended: no new dependency.** A PNG reader of our own, about 150 lines (8-bit grey, RGB and RGBA; all five filters; non-interlaced; the ancillary chunks ignored, a bad CRC refused), over **miniz's `tinfl` inflate**, which `spade_mesh_import` already compiles for the 3MF reader.

| | |
|---|---|
| Name and licence | miniz `3.1.2`, MIT, already in `THIRD_PARTY_NOTICES.md` (the 3MF reader's ZIP). Its row gains "and the PNG reader's inflate". No new row |
| Pin and vendor | unchanged: the release archive pinned by SHA-256 in `vendor/CMakeLists.txt`, fetched at configure time, not committed. **No new fetch** |
| Where | `world/mesh_import/png.cpp`, declared in `world/mesh_import.hpp` (`decode_png(bytes) -> Result<Image8>`), because miniz is private to that archive and the render library links none. That folder is Core's, so this needs your word |
| Not supported, refused by name | JPEG, 16-bit, palette and interlaced PNG. The fetch asks the NAIP service for PNG output (the `exportImage` request takes a format; **INFERRED, Interface checks it**). If the service cannot give PNG, JPEG needs a decoder: `stb_image` (public domain or MIT, one header, pinned by commit) is the candidate, and it is a new dependency and a new fetch, so your word again |
| Tests | PNGs are assembled in the test itself (miniz's `tdefl` writes them), so no image is committed; every filter type, each colour type, a truncated file, a bad CRC, a size bomb |

If you would rather keep Rendering free of decoding, the cache can hold raw RGB8 (`aerial.rgb8`) and the fetch tool decodes. I would not: the fetch tool then needs the decoder anyway.

## The cache format (agreed with Interface, who owns SP-5)

The layout is Interface's, in sections 4 and 5 of `docs/design/interface/plans/2026-11-17-geo-map-fetch-design.md` (branch `interface/geo-fetch-note`): `<root>/v1/<key>/` with `georef.yaml` (written last; its presence means complete), `height.f32` and `image.png`. It is what the terrain wants: rows north to south, columns west to east, pixel-is-area, one grid plus one image of different sizes, heights in metres above the ellipsoid, both layers covering `bbox_deg`. Rendering needs three more things in it, and does one job the other way round:

- **`sha256` of each file, and each source's `url` and `retrieved_utc`** (the lead's brief for this phase: "sources, URLs, dates and sha256"). A hash that does not match is `map_invalid`, a named refusal and plain ground.
- **Who reads.** Interface writes the cache reader (read-only, with `map_missing`) and it returns a `TerrainInput`. Rendering defines that struct in `render/terrain.hpp` in the first branch, so Interface builds against it. The reader owns every datum and geodesy step: it subtracts `anchor_ground.ellipsoid_height_m` from each height (world y = 0 is the anchor's ground), and it turns `bbox_deg` into `extent` by taking the four corners through SP-1's ENU at the anchor, east_min and east_max as the means of the two west and two east corners, north likewise. Over 1 km the corners differ from a rectangle by about 0.15 m (meridian convergence, INFERRED, a test measures it), which is below a texel's worth of error.
- **The PNG** is stored as received. The decoder is below, and the reader calls it.
- **Side limit.** Interface's 1 001-sample grid covers 1 001 m at 1 m cells. Rendering refuses a side more than one source cell over 1 000 m, so a full-size area passes and a 2 km one does not.

The cache holds no map data in git (REQ-G10); the tests' fixtures are made in code.

## Tests (no network, no committed map data)

1. **Mesh.** A ramp and a sine bump: vertex positions equal the formula at the placement; `TerrainStats`' gap is zero on a ramp and bounded on the bump; normals unit and pointing up; a side over 1 000 m and every bad input refused by name with the scene unchanged; the same bytes twice.
2. **Placement.** A known (east, north, height) lands at (x, y, −z) of the expected vertex.
3. **CPU texture.** A constant image equals the plain material's frame (to one level); a four-colour checker puts each colour on its quadrant, north up; a camera looking at an oblique corner shows perspective-correct texel positions (the case an affine interpolation fails).
4. **Ground.** With terrain attached, the ground never shows through it (a ramp that dips under `ground_y`); `ground_y` and the SDF are unchanged; a frame without terrain is bit-identical to today's goldens.
5. **GL** (`gpu`): the same checker, banded against the CPU, and a refused over-size texture.
6. **PNG** as in the decoder table.

## Order of work, each its own branch (RED test commit, then GREEN)

1. The mesh and `attach_terrain`, with the ground rule (tests 1, 2, 4). CPU only.
2. The CPU texture (test 3).
3. The GL texture and its band (test 5), a gpu-class gate.
4. The PNG reader (test 6), after your word on the folder.

None of it touches the golden frames, the agreement bands or the engine's scans (no libm or wall-clock under `engine/`).

## Asked of the lead

1. The decoder: own PNG reader over miniz, in `world/mesh_import/`. Yes or no, and if no, `stb_image` or raw RGB8 in the cache.
2. The ground rule: lower the render ground, or clip it.
3. The default shading, lambert or unlit.
4. Rendering's side of an editor message when a map is refused: the reason string is the `Result`'s; KT-4 shows it.
