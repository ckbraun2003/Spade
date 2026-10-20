# Sensor views: depth, normals, segmentation (N13, part 1) — design

Status: **proposed** (Rendering, 2026-10-20), for the lead's review. Branch `rendering/sensor-views`.

Three new `ViewBase` kinds in `render/view_mode`, so a view can show what a depth camera, a normal estimator or a segmenter would see. Cameron's "add different rendering schemas". Per-camera field channels, exported values and placed scene cameras are later (see "Not in this part").

## Rules shared by the three

A sensor view draws **geometry only, as data** (`01-techniques-and-channels.md`: "a field channel is data, not appearance").

- **What draws:** statics, dynamics, and CSG subtrees marched per pixel (`RS3`). The analytic ground plane also draws: it is the world's surface, and each mode gives it its own value. The bounded ground mesh alone would end the floor at its edge.
- **What doesn't:** lighting, shadows, the atmospheric term, the sky gradient, the horizon blend, the ground grid and overlays. A pixel no surface covers is **black**, named "no surface" in the legend; no mode below can produce black for a surface.
- **No flow layers** on a sensor base. A ViewSpec with layers is refused by name in `supports()` and `apply_view()`, as `exact_sdf` already refuses layers. `apply_view()` clears the scene's flow and field content, as it does for every view.
- **Single-sample targets only.** GL refuses a sensor base when the bound framebuffer is multisampled (one `GL_SAMPLES` query): a blended edge pixel is a value no surface has. Nothing in the repo multisamples today.
- Both backends draw all three. `exact_sdf` stays CPU-only, unchanged.

## Depth

- **What each pixel shows:** the surface's **view-space depth `z`**, in metres along the camera's forward axis: the planar depth an RGB-D camera reports, not the radial range.
- **Mapping:** through a `ColourMapId` over a stated range `[near_m, far_m]` (`ViewSpec::depth`; defaults 0.1 m, 20 m, viridis). Depths outside the range clamp to its ends, as every legend range does.
- **Refusals:** a range that isn't finite, has `near_m < 0`, or has `far_m <= near_m` is refused by name.
- **Legend:** "depth (view z)", m, `[near_m, far_m]`, the map, plus the "no surface" swatch.
- **CPU:** `FrameBuffers::depth` already holds `1/z`, perspective-correct for meshes and written by the CSG march. The analytic ground's `best_t` is already view depth. One pass after the geometry colours each covered pixel.
- **GL:**
  - The mesh shader takes `z = dot(p_world - camera, forward)` from a world-position varying, exact to float. Linearising the 24-bit depth buffer would not be.
  - The CSG shader takes `z` from its hit point, and the ground shader from its `t`.
  - Both use the shared 256-entry colour-map texture. That texture is bound today only when a flow layer exists, so it will also be bound in depth mode.
- **Parity test:** a slanted plane of known depth. Each backend's colours must equal `colour_map_at(t)`'s bytes (CPU exactly, GL within a band), and CPU against GL within a pinned band.

## Normals

- **What each pixel shows:** the surface's **unit world-space outward normal `n`** as `RGB = (n + 1) / 2`.
- **Legend:** "+X red, +Y green, +Z blue; world frame, Y up". `(n + 1)/2` is never black for a unit `n`, so black stays "no surface".
- **Which normal:** for meshes, the vertex normals interpolated perspective-correctly and renormalised per pixel. For CSG, the SDF gradient. For the ground, the plane's normal. Back faces are culled, so nothing is flipped toward the eye.
- **CPU change (the one real one):** the CPU shades at vertices and interpolates colour affinely in screen space, so it holds no per-pixel normal. In normals mode, the triangle loop interpolates `n/z` affinely, divides by the `1/z` it already has, and normalises. GL's `normalize(vNormal)` is already perspective-correct.
- **Parity test:** a box's faces give exact colours on the CPU. CPU against GL within about 1° on interior pixels.
- **Later:** camera-frame normals, an option of one matrix per frame.

## Segmentation

- **What each pixel shows:** every draw item fills flat with its **label's colour**. The spec chooses objects or parts (`ViewSpec::segment_by`). An item with no part label uses its object's.
- **Identity, which doesn't exist today.** No DrawItem, mesh or pixel carries one, and `mesh_index` is shared across items.
  - `DrawItem` gains `object_label` and `part_label` (`kNoLabel` by default).
  - `RenderScene` gains `labels` (`std::vector<std::string>`; a label is an index into it).
- **Colour from the label's NAME:** `segment_colour(name)` hashes the UTF-8 name (FNV-1a 64) to a hue, at fixed saturation and two lightness steps.
  - The same name gets the same colour in every frame, run and backend, whatever the draw order and whatever else the scene holds. Adding an object recolours nothing.
  - Unique names give instance segmentation. A shared name (e.g. "tree") gives semantic segmentation. That's the producer's choice.
  - One function on the host computes it. GL receives the colours per instance, beside the speeds, so both backends draw the same bytes (TD-9).
- **Never silent:**
  - Unlabelled items draw in a stated grey (saturation 0, which no label uses), and the legend gives their count.
  - Two different names whose colours fall within a stated distance are listed as a clash in the legend.
- **Legend:** `base_legend(spec, scene)` gives {name, colour} for each label a drawn item uses, sorted by name, plus "unlabelled (n)", "no surface", and any clashes.
- **Who labels:**
  - `scene_from_world`: "sdf node <i>" for SDF statics (world nodes have no names), and "prop <i>: <mesh_ref>".
  - `update_dynamics`: "body <i>".
  - `render_parts::bind_parts`: the object name its caller passes, with parts as "<object>/<part name>" (the model's part name, or "parts[k]").
  - The editor names its scene objects. That's Interface's follow-up (a consumers line asks for it); until then the defaults show.
- **CPU:** a flat fill per item, cheaper than shaded.
- **GL:** a per-instance colour array filled in the same regrouping as the speeds (`vInstance` indexes instances, not DrawItems), and a per-subtree uniform for CSG.
- **Tests:**
  - Two scenes with the same names in a different order give the same colours.
  - The legend's contents.
  - The unlabelled grey.
  - The clash detector.
  - CPU against GL equal on interior pixels; edges may differ by the rasterisers' rules.

## API (proposed)

- `ViewBase` gains `depth = 4`, `normals = 5` and `segmentation = 6` (`kViewBaseCount` becomes 7), named "depth", "normals", "segmentation". `DrawMode` gains the same three values.
- `ViewSpec` and `RenderOptions` gain `DepthSpec depth{near_m, far_m, colours}` and `SegmentBy segment_by {objects, parts}`.
- `DrawItem` gains `object_label` and `part_label`; `RenderScene` gains `labels`. Neither breaks designated initialisers.
- New: `BaseLegend base_legend(const ViewSpec&, const RenderScene&)` and `glm::u8vec3 segment_colour(std::string_view name)`. `apply_view()`'s return type is **unchanged**, so Interface's callers don't break.
- `bind_parts` takes the object's name.
- Consumers line: three new `DrawMode` values (a `switch` without a default will warn), the label fields, and the ask that the editor label its objects.

## supports() coverage

- `view_cases()` loops over `kViewBaseCount`, so the three bases enter the CPU and GL parity tests automatically.
- New cases: a sensor base with a layer (refused on both), a bad depth range (refused on both), and each base with no layers (drawn on both).
- A multisampled target is the draw's own refusal: `supports()` can't see the framebuffer, as with the device limits it already leaves to the draw.

## Cost

- **CPU:**
  - depth: one W×H pass;
  - normals: three interpolations and a normalise per pixel, in normals mode only;
  - segmentation: cheaper than shaded (no lighting).
  - No shadow-map pass in any sensor mode.
- **GL:**
  - one `uMode` branch in the existing mesh, CSG and ground shaders; no new pass;
  - 4 bytes per instance for the label colours, uploaded with the speeds;
  - one `GL_SAMPLES` query per sensor draw.
- **Scene:** 8 bytes per DrawItem, plus the label strings.

## Not in this part

- Exporting depth in metres or labels as integers (the field and ID channels).
- Camera-frame normals.
- Placed scene cameras.
- The Vulkan technique.

The per-pixel value is computed first and coloured last on both backends, so a later channel export reads the same values.

## Decisions for the lead

1. **Colour by name hash** (stable across edits; clashes reported), or by label index (always distinct, but colours shift when an object is added). Recommended: by name.
2. **Refuse sensor bases on a multisampled GL target.** Recommended: yes.
3. **The analytic ground draws in sensor modes.** Recommended: yes.
4. **Depth defaults** 0.1 m to 20 m, viridis. The editor would set them from the scene's size.
