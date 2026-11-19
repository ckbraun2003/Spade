#pragma once
// ---------------------------------------------------------------------------
// sprite_kit -- D182 S3 (the lead approved, 2026-11-19): thin builders that make KAT's three constant-pixel markers from render/sprites.hpp's primitives.
// Spade holds NO palette and no state: every colour is an argument, the state is the caller's choice (it passes `normal`, `hover` or `occupied`), and each
// builder returns an ordinary WorldSprite the caller may change (slots, cull_outside_view) before it goes in RenderScene::sprites. The numbers are KAT's
// (the editor's drawing spec B-L2, B-L3, B-V2), logical pixels, so RenderOptions::ui_scale scales them.
// ---------------------------------------------------------------------------

#include <glm/vec3.hpp>

#include "render/sprites.hpp"

namespace spade::render {

enum class MarkKind : uint8_t { crash, lap, time_limit };
enum class ReticleState : uint8_t { normal, hover, occupied };

// A drone seen as a speck: a 6 px disc of `fill` inside a 1.5 px border (a pen centred on a circle of radius 3.75, so the lit width is 9 px), anti-aliased,
// drawn only while the drone's own span is under 12 px (visible_if: 0.162 m along the camera's right, 12 px). The caller passes the drone's world position.
[[nodiscard]] WorldSprite make_drone_disc(glm::dvec3 anchor, SpriteColour fill, SpriteColour border);

// An end-of-run mark in a 12 px box (6 px each side of the anchor) with a 2 px pen, anti-aliased, no fill: `crash` is an X (two diagonals, square caps),
// `lap` a ring (diameter 12), `time_limit` a square outline (12 x 12).
[[nodiscard]] WorldSprite make_mark(MarkKind kind, glm::dvec3 anchor, SpriteColour colour);

// A spawn point: a circle of radius 9 and four ticks from radius 13 in to 7, 1.2 px, in `reticle`, all hard-edged. `hover` draws a halo FIRST under it (radius 13,
// a 1.5 px `accent` stroke and an `accent` fill at alpha 0x22); `occupied` draws a disc AFTER the ticks (radius 4, a 1 px `ok` stroke and an `ok` fill at
// alpha 0x44). `normal` is the reticle alone.
[[nodiscard]] WorldSprite make_spawn_reticle(glm::dvec3 anchor, ReticleState state, SpriteColour reticle, SpriteColour accent, SpriteColour ok);

}  // namespace spade::render
