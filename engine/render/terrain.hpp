#pragma once

// ---------------------------------------------------------------------------
// The geo terrain's mesh (SP-4, KAT's geo plan G2; docs/design/rendering/plans/2026-11-17-geo-terrain-design.md): a
// render-only surface made from a decoded height grid, drawn under the scene's other geometry. It is for the eye only:
// it adds no collision (REQ-G16), nothing in the simulation reads it, and the physics ground stays the world's own.
//
// The map arrives already decoded and already in metres from the anchor (world/terrain_input.hpp, the cache reader's
// return type): this module reads no file, resolves no URL and does no geodesy.
//
// attach_terrain() appends ONE draw item to scene.statics, in its own label ("terrain") and its own material, and:
//   * places it at world x = east, y = height from the anchor, z = -north (x east, y up, z south; REQ-G05). The heights
//     are NOT corrected for the earth's curve, so the map agrees with the flat physics ground where the land is flat;
//   * cuts it to at most `max_cells` cells a side (default 128: 32 768 triangles at most), each vertex the source grid
//     sampled bilinearly at its position and clamped at the edge, its normal from central differences of the vertex
//     heights; the stats name the largest gap to the source, so a decimation is never silent;
//   * lowers the render copy of each FLOOR plane (a ground plane whose normal is within about 2.5 degrees of +y) to just
//     under the map's lowest vertex, when it would otherwise hide part of the map. Only the drawing moves: scene.ground_y,
//     the SDF and the physics ground are untouched (REQ-G16). When a floor is lowered the sun's static shadow map is
//     rebuilt, since a floor is one of its casters;
//   * lets the terrain RECEIVE the sun's shadow and cast none (DrawItem::casts_shadow): a kilometre of map would waste
//     the shadow map's texels, and a flat map shades nothing onto a drone.
//
// A map that cannot be drawn is REFUSED BY NAME and the scene is left exactly as it was: the scene then draws plain
// ground (REQ-G14), and the caller shows the reason (REQ-G15). The image is validated here and drawn by a later branch
// (the planar texture).
//
// Deterministic: a fixed order, no RNG, no clock; every coordinate computation is in double and the result rounded to
// float once, so every platform builds the same bytes.
// ---------------------------------------------------------------------------

#include <cstdint>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "render/scene.hpp"
#include "world/terrain_input.hpp"

namespace spade::render {

struct TerrainSpec {
    // The most cells a side of the mesh gets; a source with fewer samples is used one to one. 0 is refused.
    uint32_t max_cells = 128;
    // lambert (the default, L85b: the map receives the drone's shadow) or unlit (the map as it is, no shadow).
    bool lit = true;
    // The map's colour until the aerial texture (a later branch) replaces it: salt-flat white.
    glm::vec3 base_color{0.80f, 0.78f, 0.72f};
    // Draw the map's aerial image over the mesh (the planar texture, SP-4): attach_terrain puts it in scene.planar_texture at the map's extent and marks the terrain's
    // item. True by default: a map carries its image (attach_terrain refuses one under 2 x 2), so there is never a missing image to draw; false draws the flat colour above. The scene holds ONE
    // planar texture: a second textured map replaces the first.
    bool textured = true;
};

struct TerrainStats {
    uint32_t cells_x = 0, cells_z = 0;
    uint32_t triangles = 0;
    // The largest |mesh - source| over the cell centres, in metres: the mesh's height there (the mean of its diagonal's
    // ends) against the source grid sampled at the same point. The edge clamp counts.
    float max_gap_m = 0.0f;
    // The lowest vertex's world y: the floor rule's input.
    float lowest_y_m = 0.0f;
    // How far the lowest lowered floor went, in metres, and how many floors were lowered. 0 and 0 when none was.
    float ground_lowered_m = 0.0f;
    uint32_t planes_lowered = 0;
};

// How far under the map's lowest vertex a lowered floor sits, in metres.
inline constexpr float kTerrainGroundMargin = 0.02f;

// The side beyond which a map is refused (REQ-G07), in metres, plus one source cell of tolerance: Interface's 1 001-sample
// grid at 1 m cells covers 1 001 m and passes.
inline constexpr double kTerrainMaxSideM = 1000.0;

[[nodiscard]] Result<TerrainStats> attach_terrain(RenderScene& scene, const world::TerrainInput& map,
                                                   const TerrainSpec& spec = {});

}  // namespace spade::render
