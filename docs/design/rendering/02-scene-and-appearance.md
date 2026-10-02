# Rendering — scene representation and appearance

**Owner:** Rendering. **Normative.** How a world becomes something a technique can draw, and what a world's appearance means. The world-file format itself (keys, versions, migration) is Core's. Rendering owns what the appearance fields mean (`RS5`, split).

## The render scene (`RS2`)

`RenderScene` is the renderer's own view of a world: meshes, draw items, a material palette, lighting, ground planes, bounds, and an optional baked shadow map. It is built once per world and roster by `scene_from_world()`. Per frame, only body poses change.

| Draw item | Source | Rebuilt |
|---|---|---|
| Tessellated SDF primitive | each primitive leaf under unions | at world load |
| Meshed CSG subtree | each subtract, intersect or smooth-union subtree | at world load |
| Static prop mesh | a world's `props` | at world load |
| Body mesh | the body's visual mesh, instanced per live body | per frame, transform only |

A rebuild from a new world description must stay cheap and supported, so an editor can swap scenes without restarting.

**Target:** the scene is assembled from the published frame state (poses, plus fields a camera requests), not from simulation internals.

## Geometry (`RS3`)

- **Unions are free.** Each primitive is tessellated exactly at a fixed per-kind resolution, and the depth buffer resolves the union.
- **CSG subtrees are meshed once** by surface nets over the subtree's bounds, at a fixed resolution from a renderer-side table. Resolution is never a world-file field and never adaptive, so output stays byte-reproducible.
- **Unbounded primitives** (plane, heightfield) tessellate as a bounded grid fitted to the world bounds (`PA-5`).
- **A standalone plane is also an analytic infinite ground** (`SR-17`). It is drawn per pixel in the background pass, alongside the tessellated grid, never instead of it. A plane inside a subtract is a cutting half-space, not a floor. The two must agree bit-for-bit on the same ground, because both call `transform_normal()` and `shade_vertex_color()` on the same inputs.
- **Meshes** (glTF 2.0: positions, indices, per-primitive material) are render-only. The engine is handed loaded geometry and never resolves an asset ID itself. A missing or malformed asset draws a conspicuous fallback marker and warns once, never a blank (`RS7`).
- **Collision truth is the SDF** (`RS4`). Meshes never collide. The agreement test (`03-verification.md`) is what keeps the visual world and the physical world from disagreeing.

## Materials and light (`RS6`, `RS6a`)

- **A material** is an RGBA base colour plus one shading flag: `lambert`, `unlit` or `emissive`. Alpha exists only for envelope and marker geometry, drawn in a declared, order-stable pass. No textures, no PBR.
- **The light rig** is one directional sun (direction, colour, intensity), a flat ambient term, and a two-colour sky gradient drawn as the background.
- **The sun convention** (`RND-1`): `sun_direction` points from the scene toward the sun, and every path shades with `N·L = dot(n, +sun_direction)`. The world-level default `(0.4, 0.8, 0.6)` is above the horizon.
- **Shadows:** one orthographic sun shadow map fitted to the scene bounds, baked at world load for static geometry. Beyond the bounds the infinite ground is unshadowed. That is accepted, and it must never be hidden by the atmospheric term (`SR-17` clause 6).
- **The atmospheric term** (`SR-17a`): every shaded surface is blended toward the sky colour along its own view ray by its distance from the eye. It is one pure function of three per-pixel values (shaded colour, hit distance, sky colour along that ray), so it is shading rather than post-processing, and `RS15`'s exclusion does not apply. It must be:
  - applied per pixel, never per vertex;
  - rational, with no transcendentals;
  - written `base + (other - base) * t`, so it is exact at strength 0.

  The engine default is strength 0.

## Appearance in the world file (`RS5`, split with Core)

The world file carries `materials`, `lighting` and `props` (schema v2).
- **Per-node material** is a parallel host-only array, not the SDF node's padding word (`PA-2`). Appearance stays a render concern, and the SDF program stays a physics artifact.
- **Props** are collision-free visual instances: mesh, pose with uniform scale, material. They are the one place a visual-only object exists.
- A v1 file loads with the default material, the default lighting and no props.

## Overlays (`PA-4`)

The ground grid, world-bounds box and spawn markers are presentation overlays, switched by `RenderOptions::overlays`. They never take part in any agreement comparison. The analytic infinite grid is not an overlay: it is drawn in the background pass, in shaded mode only.
