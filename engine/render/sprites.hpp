#pragma once
// ---------------------------------------------------------------------------
// sprites -- D182 S3 (KAT's drawing spec P2; the lead approved this header, 2026-11-19): world-anchored, CONSTANT-PIXEL sprites. A sprite is an anchor in
// world space and a list of 2D shapes in pixels from its projected point, so a marker keeps its size however far it is. RenderScene::sprites draws after
// the flow polylines, in submission order (a sprite's shapes in theirs), clipped to the frame. The CPU renderer is the reference; GL is held to it.
//
// Spade holds NO palette and no state: every colour is an argument, and "hover" or "occupied" is the caller's choice of shape list (render/sprite_kit.hpp
// builds KAT's three from colours it is given).
//
// CONVENTIONS (the spec's P0): sizes are LOGICAL pixels, multiplied by RenderOptions::ui_scale (every offset, radius, stroke). Offsets are from the
// projected anchor, y DOWN. The projection is the camera's own pinhole: an anchor is not projected when its camera-space z >= -1e-9, there is NO near or far
// cull, and a pixel centre is at +0.5 (an anchor at (16.5, 16.5) is the centre of pixel (16, 16)). Colour is 8-bit sRGB, straight alpha, source-over, the
// byte written as given (no gamma). A stroke is centred on its path, half inside and half outside. aa = false: a pixel is covered iff its CENTRE is inside
// the shape (a hard edge; it matches Qt's non-anti-aliased output to within a pixel at the edge, no better); aa = true blends by coverage. Sprites draw on
// the viewer slot only unless `slots` says otherwise.
// ---------------------------------------------------------------------------

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include "core/error.hpp"

namespace spade::render {

enum class SpriteShapeKind : uint8_t { circle, rect, line, polygon, arc };
// The ends of a line or an arc. square = Qt's default pen: the stroke runs half its width past the end (the crash X relies on it).
enum class SpriteCap : uint8_t { square, flat, round };

struct SpriteColour {
    uint8_t r = 0, g = 0, b = 0, a = 255;
};

struct SpriteShape {
    SpriteShapeKind kind = SpriteShapeKind::circle;
    // circle: centre (x0, y0), radius r.  rect: corner (x0, y0), size (x1, y1) = (w, h), both >= 0.
    // line: (x0, y0) to (x1, y1), a stroke only.  arc: centre (x0, y0), radius r, from start_deg through span_deg (0 = 3 o'clock, positive =
    // counter-clockwise as the viewer sees it; |span| >= 360 is the whole circle), a stroke only.  polygon: `points` (3 to kMaxSpritePolygonPoints).
    float x0 = 0.0f, y0 = 0.0f, x1 = 0.0f, y1 = 0.0f;
    float r = 0.0f, start_deg = 0.0f, span_deg = 360.0f;
    std::vector<glm::vec2> points;
    std::optional<SpriteColour> fill, stroke;  // a circle, rect or polygon needs at least one; a line or an arc needs a stroke
    float stroke_px = 1.0f;                    // logical px, centred on the path; > 0 when there is a stroke
    bool aa = true;
    SpriteCap cap = SpriteCap::square;
};

struct WorldSprite {
    glm::dvec3 anchor{0.0};                  // world metres
    std::vector<SpriteShape> shapes;         // 1 to kMaxSpriteShapes, drawn in this order
    uint32_t slots = 1u;                     // a mask of RenderOptions::slot values; slot 0 only by default (polylines default to every slot)
    bool cull_outside_view = false;          // true: drop the sprite when its anchor, rounded to a pixel, is outside the frame
    // visible_if: with visible_max_span_px > 0 the sprite draws only when 2 * hypot(proj(anchor + visible_half_m along the camera's right) -
    // proj(anchor)) < visible_max_span_px; if either point does not project it does not draw. KAT's drone disc: 0.162 m and 12 px.
    float visible_half_m = 0.0f;
    float visible_max_span_px = 0.0f;        // 0 = always
};

inline constexpr size_t kMaxSprites = 65536;           // per scene
inline constexpr size_t kMaxSpriteShapes = 16;         // per sprite
inline constexpr size_t kMaxSpritePolygonPoints = 64;  // per polygon

// A sprite that cannot be drawn is refused BY NAME before any pixel is written, the same on both renderers: "sprites[i].shapes[j]: ...".
[[nodiscard]] Result<void> validate_sprite(const WorldSprite& sprite);
[[nodiscard]] Result<void> validate_sprite_budget(std::span<const WorldSprite> sprites);

// What draw_sprite_shapes writes to. frame_bgrx: B, G, R, X bytes of an opaque frame, blended onto what is there. overlay_rgba: R, G, B, A bytes of a
// transparent overlay holding PREMULTIPLIED colour (source-over onto the overlay), which GL composites over its frame with (ONE, ONE_MINUS_SRC_ALPHA).
enum class SpriteTarget : uint8_t { frame_bgrx, overlay_rgba };

// The reference rasteriser: draws `shapes` (px offsets from (anchor_x, anchor_y), a point in FRAME px, pixel centres at +0.5) into a B, G, R, X frame of
// width x height with RenderOptions::ui_scale applied, source-over, clipped to the frame. render() calls it for every sprite that projects; the shapes
// were validated. aa = true samples 4 x 4 per pixel, false takes the pixel centre.
void draw_sprite_shapes(std::span<uint8_t> pixels, uint32_t width, uint32_t height, double anchor_x, double anchor_y, float ui_scale,
                        std::span<const SpriteShape> shapes, SpriteTarget target = SpriteTarget::frame_bgrx);

}  // namespace spade::render
