# spade::render_parts — drawing a model's parts (RND-9)

**Owner:** Rendering. **Ruling:** `RND-9` (the lead, 2026-10-13). **Users:** `flow_movie` (D1), the sandbox (D2), Kat's host. **Review:** the lead; Core reviews the CMake and the module edge.

## What it is

A small engine library, `spade_render_parts` (`spade::render_parts`), linking `spade::render` and `spade::vehicles`, both below it. It turns a model's parts, posed by Core's `posed_parts()`, into render-scene items. Nothing in `render/` or `vehicles/` includes it.

```cpp
namespace spade::render_parts {
struct PartsLook {                 // materials: indices into the scene's palette
    uint32_t body = 0, rotor = 0, porous = 0;
    bool rotor_parts_are_walls = false;  // the actuator tiers: the solver never sees a rotor part (P5 revisits)
};
// Once per model, in each PART frame: a mesh part's triangles; a shape part
// split as scene_from_world() splits a world (render/csg_mesh.hpp's
// split_program()): union-primitive leaves tessellated, each CSG root meshed
// over part_bounds() and kept as its subtree program for the per-pixel march.
[[nodiscard]] Result<PreparedParts> prepare_parts(const vehicles::ModelType& model);
// One frame: appends each posed part's meshes and draw items (local_to_world =
// the part's world pose x the piece's own), and a CsgSubtree per CSG piece
// (its program composed into the world, as posed_program() composes, and its
// bounds the posed box's). flow_wall: a wall part on the body; never a porous
// part; a rotor part only if PartsLook says so.
[[nodiscard]] Result<void> bind_parts(const PreparedParts& prepared, const vehicles::PosedParts& posed,
                                      const PartsLook& look, render::RenderScene& scene);
}
```

The items go into `statics`, because a `CsgSubtree` indexes `statics` (RS3) and `flow_movie` builds its scene per frame. A scene built once and posed per frame (`RS2`, D2's live view) needs moving CSG subtrees, which no renderer has today. That's a follow-up when D2 needs it, not this plan.

## Tests (red first, then green)

1. **The reference quad's parts draw where `posed_parts()` puts them.** Compile the reference quad and pose it tilted, at a height. Every vertex of every bound mesh, taken to the world, lies on the posed field within the tessellation's band: `|sample_parts(model, posed, x).distance| <= band`. A CPU frame of the bound parts matches a CPU ray-march of `posed_program()` within a pinned silhouette band (`RS4`).
2. **flow_wall:** wall parts on the body set it; porous parts never; rotor parts only by `PartsLook`. On a CPU frame with a `walls_only` C_p colouring, only those items take the colour map (`RND-8`).
3. **A mesh part** binds its own triangles at its pose. **A CSG shape part** binds a `CsgSubtree` whose program samples as `sample_parts()` does at world points, inside posed bounds that hold the surface.
4. **Refusals by name:** a posed part that isn't in the prepared model; a scene material index out of range.
5. **The edge:** a source scan that `render/` and `vehicles/` include nothing from `render_parts/`, as `PartPose.VehiclesAndWorldIncludeNothingAboveThem` does for `vehicles/`.

The fixture: test 1 needs the reference quad's `AirframeSpec`, which lives only in `tests/test_airframe_compile.cpp`. It moves to a shared test header, `tests/reference_airframe.hpp`, unchanged. That's a test-only move in Core's file, for Core to review. Tests 2–4 use a small hand-built model that has every kind: a mesh part, a CSG shape, a porous part and a rotor part. The reference quad has only primitive walls.

## Shipping

Installed with its header, plus a consumers line (the lead's `consumers.md`). gcc `--compile` covers it as an engine library, which the leg builds. Then `flow_movie --case` (D1) calls it, and the sandbox follows with D2.
