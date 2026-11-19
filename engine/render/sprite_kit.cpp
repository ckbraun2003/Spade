#include "render/sprite_kit.hpp"

#include <utility>

namespace spade::render {
namespace {

[[nodiscard]] SpriteShape circle(float r, std::optional<SpriteColour> fill, std::optional<SpriteColour> stroke, float stroke_px, bool aa) {
    SpriteShape s;
    s.kind = SpriteShapeKind::circle;
    s.r = r;
    s.fill = fill;
    s.stroke = stroke;
    s.stroke_px = stroke_px;
    s.aa = aa;
    return s;
}

[[nodiscard]] SpriteShape line(float x0, float y0, float x1, float y1, SpriteColour colour, float stroke_px, bool aa) {
    SpriteShape s;
    s.kind = SpriteShapeKind::line;
    s.x0 = x0;
    s.y0 = y0;
    s.x1 = x1;
    s.y1 = y1;
    s.stroke = colour;
    s.stroke_px = stroke_px;
    s.aa = aa;
    s.cap = SpriteCap::square;  // Qt's default pen, which the crash X and the reticle ticks were drawn with
    return s;
}

[[nodiscard]] SpriteColour with_alpha(SpriteColour c, uint8_t a) {
    c.a = a;
    return c;
}

}  // namespace

WorldSprite make_drone_disc(glm::dvec3 anchor, SpriteColour fill, SpriteColour border) {
    constexpr float kFillPx = 6.0f, kBorderPx = 1.5f;
    WorldSprite sprite;
    sprite.anchor = anchor;
    sprite.shapes.push_back(circle(0.5f * (kFillPx + kBorderPx), fill, border, kBorderPx, true));
    sprite.visible_half_m = 0.162f;
    sprite.visible_max_span_px = 12.0f;
    return sprite;
}

WorldSprite make_mark(MarkKind kind, glm::dvec3 anchor, SpriteColour colour) {
    constexpr float kHalf = 6.0f, kPen = 2.0f;
    WorldSprite sprite;
    sprite.anchor = anchor;
    switch (kind) {
        case MarkKind::crash:
            sprite.shapes.push_back(line(-kHalf, -kHalf, kHalf, kHalf, colour, kPen, true));
            sprite.shapes.push_back(line(kHalf, -kHalf, -kHalf, kHalf, colour, kPen, true));
            break;
        case MarkKind::lap:
            sprite.shapes.push_back(circle(kHalf, std::nullopt, colour, kPen, true));
            break;
        case MarkKind::time_limit: {
            SpriteShape box;
            box.kind = SpriteShapeKind::rect;
            box.x0 = -kHalf;
            box.y0 = -kHalf;
            box.x1 = 2.0f * kHalf;
            box.y1 = 2.0f * kHalf;
            box.stroke = colour;
            box.stroke_px = kPen;
            sprite.shapes.push_back(std::move(box));
            break;
        }
    }
    return sprite;
}

WorldSprite make_spawn_reticle(glm::dvec3 anchor, ReticleState state, SpriteColour reticle, SpriteColour accent, SpriteColour ok) {
    constexpr float kRadius = 9.0f, kTickOuter = 13.0f, kTickInner = 7.0f, kStroke = 1.2f, kHalo = 13.0f, kDisc = 4.0f;
    WorldSprite sprite;
    sprite.anchor = anchor;
    if (state == ReticleState::hover) sprite.shapes.push_back(circle(kHalo, with_alpha(accent, 0x22), accent, 1.5f, false));
    sprite.shapes.push_back(circle(kRadius, std::nullopt, reticle, kStroke, false));
    sprite.shapes.push_back(line(kTickInner, 0.0f, kTickOuter, 0.0f, reticle, kStroke, false));
    sprite.shapes.push_back(line(-kTickInner, 0.0f, -kTickOuter, 0.0f, reticle, kStroke, false));
    sprite.shapes.push_back(line(0.0f, kTickInner, 0.0f, kTickOuter, reticle, kStroke, false));
    sprite.shapes.push_back(line(0.0f, -kTickInner, 0.0f, -kTickOuter, reticle, kStroke, false));
    if (state == ReticleState::occupied) sprite.shapes.push_back(circle(kDisc, with_alpha(ok, 0x44), ok, 1.0f, false));
    return sprite;
}

}  // namespace spade::render
