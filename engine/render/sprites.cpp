#include "render/sprites.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "core/fp32_math.hpp"

namespace spade::render {
namespace {

using Code = spade::Code;

[[nodiscard]] Result<void> refuse(const std::string& what) {
    return std::unexpected(Error{Code::invalid_argument, what});
}

[[nodiscard]] bool finite(float v) noexcept { return std::isfinite(v); }

}  // namespace

Result<void> validate_sprite(const WorldSprite& sprite) {
    if (!std::isfinite(sprite.anchor.x) || !std::isfinite(sprite.anchor.y) || !std::isfinite(sprite.anchor.z)) {
        return refuse("anchor is not finite");
    }
    if (!finite(sprite.visible_half_m) || sprite.visible_half_m < 0.0f || !finite(sprite.visible_max_span_px) || sprite.visible_max_span_px < 0.0f) {
        return refuse("visible_half_m and visible_max_span_px must be finite and >= 0");
    }
    if (sprite.visible_max_span_px > 0.0f && !(sprite.visible_half_m > 0.0f)) {
        return refuse("visible_max_span_px is set but visible_half_m is not > 0");
    }
    if (sprite.shapes.empty() || sprite.shapes.size() > kMaxSpriteShapes) {
        return refuse("shapes: " + std::to_string(sprite.shapes.size()) + " shapes; a sprite has 1 to " + std::to_string(kMaxSpriteShapes));
    }
    for (size_t j = 0; j < sprite.shapes.size(); ++j) {
        const SpriteShape& s = sprite.shapes[j];
        const std::string at = "shapes[" + std::to_string(j) + "]: ";
        if (!finite(s.x0) || !finite(s.y0) || !finite(s.x1) || !finite(s.y1) || !finite(s.r) || !finite(s.start_deg) || !finite(s.span_deg) ||
            !finite(s.stroke_px)) {
            return refuse(at + "a value is not finite");
        }
        const bool line_like = s.kind == SpriteShapeKind::line || s.kind == SpriteShapeKind::arc;
        if (!s.fill && !s.stroke) return refuse(at + "needs a fill or a stroke");
        if (line_like && !s.stroke) return refuse(at + "a line or an arc needs a stroke");
        if (line_like && s.fill) return refuse(at + "a line or an arc has no fill; give it a stroke");
        if (s.stroke && !(s.stroke_px > 0.0f)) return refuse(at + "stroke_px must be > 0 when there is a stroke");
        switch (s.kind) {
            case SpriteShapeKind::circle:
            case SpriteShapeKind::arc:
                if (s.r < 0.0f) return refuse(at + "r must be >= 0");
                break;
            case SpriteShapeKind::rect:
                if (s.x1 < 0.0f || s.y1 < 0.0f) return refuse(at + "a rect's width and height (x1, y1) must be >= 0");
                break;
            case SpriteShapeKind::polygon:
                if (s.points.size() < 3u || s.points.size() > kMaxSpritePolygonPoints) {
                    return refuse(at + "polygon has " + std::to_string(s.points.size()) + " points; a polygon has 3 to " +
                                  std::to_string(kMaxSpritePolygonPoints));
                }
                for (const glm::vec2& p : s.points) {
                    if (!finite(p.x) || !finite(p.y)) return refuse(at + "a polygon point is not finite");
                }
                break;
            case SpriteShapeKind::line:
                break;
        }
    }
    return {};
}

Result<void> validate_sprite_budget(std::span<const WorldSprite> sprites) {
    if (sprites.size() > kMaxSprites) {
        return refuse("sprites: " + std::to_string(sprites.size()) + " sprites exceed the " + std::to_string(kMaxSprites) +
                      " one scene draws; draw fewer");
    }
    for (size_t i = 0; i < sprites.size(); ++i) {
        if (Result<void> valid = validate_sprite(sprites[i]); !valid) {
            return refuse("sprites[" + std::to_string(i) + "]." + valid.error().context);
        }
    }
    return {};
}

}  // namespace spade::render

// ---------------------------------------------------------------------------
// The reference rasteriser. Everything is in frame pixels relative to the anchor, y down, scaled by ui_scale. A shape is a coverage test "is this point
// inside the fill / the stroke"; a pixel's coverage is the share of its 4 x 4 samples inside (aa) or whether its centre is (hard edge).
// ---------------------------------------------------------------------------
namespace spade::render {
namespace {

struct P2 {
    float x, y;
};

[[nodiscard]] float len(P2 v) noexcept { return std::sqrt(v.x * v.x + v.y * v.y); }
[[nodiscard]] float cross(P2 a, P2 b) noexcept { return a.x * b.y - a.y * b.x; }
[[nodiscard]] float dot(P2 a, P2 b) noexcept { return a.x * b.x + a.y * b.y; }
[[nodiscard]] float absf(float v) noexcept { return v < 0.0f ? -v : v; }

[[nodiscard]] float dist_to_segment(P2 p, P2 a, P2 b) noexcept {
    const P2 ab{b.x - a.x, b.y - a.y};
    const float l2 = dot(ab, ab);
    float t = l2 > 0.0f ? dot(P2{p.x - a.x, p.y - a.y}, ab) / l2 : 0.0f;
    t = std::min(1.0f, std::max(0.0f, t));
    return len(P2{p.x - (a.x + ab.x * t), p.y - (a.y + ab.y * t)});
}

// One shape, scaled: the two coverage tests and a bounding box (relative to the anchor) that holds everything either can cover.
struct ScaledShape {
    SpriteShape src;
    float hw = 0.0f;  // half the stroke, scaled
    float x0 = 0, y0 = 0, x1 = 0, y1 = 0, r = 0;
    float bx0 = 0, by0 = 0, bx1 = 0, by1 = 0;
    std::vector<P2> poly;
    P2 dir0{1, 0}, dir1{1, 0};  // arc: unit vectors (y up) at the start and the end of the sweep
    bool full = true;
    float span = 360.0f;

    [[nodiscard]] bool in_fill(P2 p) const noexcept {
        switch (src.kind) {
            case SpriteShapeKind::circle: {
                const float dx = p.x - x0, dy = p.y - y0;
                return dx * dx + dy * dy <= r * r;
            }
            case SpriteShapeKind::rect:
                return p.x >= x0 && p.x <= x0 + x1 && p.y >= y0 && p.y <= y0 + y1;
            case SpriteShapeKind::polygon: {
                bool in = false;
                const size_t n = poly.size();
                for (size_t i = 0, j = n - 1u; i < n; j = i++) {
                    const P2 a = poly[i], b = poly[j];
                    if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) in = !in;
                }
                return in;
            }
            default:
                return false;
        }
    }

    [[nodiscard]] bool in_sweep(P2 q) const noexcept {  // q is y up, from the arc's centre
        if (full) return true;
        if (span <= 180.0f) return cross(dir0, q) >= 0.0f && cross(q, dir1) >= 0.0f;
        return !(cross(dir1, q) >= 0.0f && cross(q, dir0) >= 0.0f);
    }

    [[nodiscard]] bool in_stroke(P2 p) const noexcept {
        switch (src.kind) {
            case SpriteShapeKind::circle: {
                const float d = len(P2{p.x - x0, p.y - y0});
                return absf(d - r) <= hw;
            }
            case SpriteShapeKind::rect: {
                const bool outer = p.x >= x0 - hw && p.x <= x0 + x1 + hw && p.y >= y0 - hw && p.y <= y0 + y1 + hw;
                const bool inner = x1 > 2.0f * hw && y1 > 2.0f * hw && p.x > x0 + hw && p.x < x0 + x1 - hw && p.y > y0 + hw && p.y < y0 + y1 - hw;
                return outer && !inner;
            }
            case SpriteShapeKind::polygon: {
                const size_t n = poly.size();
                for (size_t i = 0, j = n - 1u; i < n; j = i++) {
                    if (dist_to_segment(p, poly[j], poly[i]) <= hw) return true;
                }
                return false;
            }
            case SpriteShapeKind::line: {
                const P2 a{x0, y0}, b{x1, y1};
                const P2 ab{b.x - a.x, b.y - a.y};
                const float l = len(ab);
                if (src.cap == SpriteCap::round) return dist_to_segment(p, a, b) <= hw;
                if (!(l > 1.0e-6f)) {  // a point: a square cap is a square, a flat one nothing
                    return src.cap == SpriteCap::square && absf(p.x - a.x) <= hw && absf(p.y - a.y) <= hw;
                }
                const P2 u{ab.x / l, ab.y / l};
                const P2 d{p.x - a.x, p.y - a.y};
                const float along = dot(d, u), perp = absf(cross(u, d));
                const float lo = src.cap == SpriteCap::square ? -hw : 0.0f, hi = src.cap == SpriteCap::square ? l + hw : l;
                return along >= lo && along <= hi && perp <= hw;
            }
            case SpriteShapeKind::arc: {
                const P2 q{p.x - x0, -(p.y - y0)};
                const float d = len(q);
                if (absf(d - r) <= hw && in_sweep(q)) return true;
                if (src.cap == SpriteCap::flat || full) return false;
                const P2 e0{dir0.x * r, dir0.y * r}, e1{dir1.x * r, dir1.y * r};
                if (src.cap == SpriteCap::round) {
                    return len(P2{q.x - e0.x, q.y - e0.y}) <= hw || len(P2{q.x - e1.x, q.y - e1.y}) <= hw;
                }
                const P2 out0{dir0.y, -dir0.x}, out1{-dir1.y, dir1.x};  // away from the sweep, along the tangent
                const auto cap_box = [&](P2 e, P2 out, P2 radial) {
                    const P2 w{q.x - e.x, q.y - e.y};
                    const float along = dot(w, out), side = absf(dot(w, radial));
                    return along >= 0.0f && along <= hw && side <= hw;
                };
                return cap_box(e0, out0, dir0) || cap_box(e1, out1, dir1);
            }
        }
        return false;
    }
};

[[nodiscard]] ScaledShape scale_shape(const SpriteShape& sh, float s) {
    ScaledShape o;
    o.src = sh;
    o.hw = sh.stroke ? 0.5f * sh.stroke_px * s : 0.0f;
    o.x0 = sh.x0 * s;
    o.y0 = sh.y0 * s;
    o.x1 = sh.x1 * s;
    o.y1 = sh.y1 * s;
    o.r = sh.r * s;
    const float pad = o.hw * 1.5f + 2.0f;  // a square cap's corner is hw * sqrt(2) out
    switch (sh.kind) {
        case SpriteShapeKind::circle:
        case SpriteShapeKind::arc:
            o.bx0 = o.x0 - o.r - pad, o.bx1 = o.x0 + o.r + pad, o.by0 = o.y0 - o.r - pad, o.by1 = o.y0 + o.r + pad;
            break;
        case SpriteShapeKind::rect:
            o.bx0 = o.x0 - pad, o.bx1 = o.x0 + o.x1 + pad, o.by0 = o.y0 - pad, o.by1 = o.y0 + o.y1 + pad;
            break;
        case SpriteShapeKind::line:
            o.bx0 = std::min(o.x0, o.x1) - pad, o.bx1 = std::max(o.x0, o.x1) + pad;
            o.by0 = std::min(o.y0, o.y1) - pad, o.by1 = std::max(o.y0, o.y1) + pad;
            break;
        case SpriteShapeKind::polygon: {
            o.bx0 = o.by0 = 1.0e30f;
            o.bx1 = o.by1 = -1.0e30f;
            for (const glm::vec2& p : sh.points) {
                o.poly.push_back(P2{p.x * s, p.y * s});
                o.bx0 = std::min(o.bx0, p.x * s - pad), o.bx1 = std::max(o.bx1, p.x * s + pad);
                o.by0 = std::min(o.by0, p.y * s - pad), o.by1 = std::max(o.by1, p.y * s + pad);
            }
            break;
        }
    }
    if (sh.kind == SpriteShapeKind::arc) {
        constexpr float kRad = 0.017453292f;
        float a0 = sh.start_deg, sp = sh.span_deg;
        if (sp < 0.0f) {
            a0 += sp;
            sp = -sp;
        }
        o.span = sp;
        o.full = sp >= 360.0f;
        o.dir0 = P2{spade::math::cos32(a0 * kRad), spade::math::sin32(a0 * kRad)};
        o.dir1 = P2{spade::math::cos32((a0 + sp) * kRad), spade::math::sin32((a0 + sp) * kRad)};
    }
    return o;
}

void blend_overlay(uint8_t* px, SpriteColour c, float cov) {  // px is R, G, B, A, premultiplied
    const float a = cov * static_cast<float>(c.a) / 255.0f;
    if (!(a > 0.0f)) return;
    const auto mix = [&](uint8_t behind, float src) {
        const float v = src * a + static_cast<float>(behind) * (1.0f - a);
        return static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, v + 0.5f)));
    };
    px[0] = mix(px[0], static_cast<float>(c.r));
    px[1] = mix(px[1], static_cast<float>(c.g));
    px[2] = mix(px[2], static_cast<float>(c.b));
    px[3] = mix(px[3], 255.0f);
}

void blend(uint8_t* px, SpriteColour c, float cov) {  // px is B, G, R, X
    const float a = cov * static_cast<float>(c.a) / 255.0f;
    if (!(a > 0.0f)) return;
    const auto mix = [&](uint8_t behind, uint8_t src) {
        const float v = static_cast<float>(behind) * (1.0f - a) + static_cast<float>(src) * a;
        return static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, v + 0.5f)));
    };
    px[0] = mix(px[0], c.b);
    px[1] = mix(px[1], c.g);
    px[2] = mix(px[2], c.r);
}

}  // namespace

void draw_sprite_shapes(std::span<uint8_t> pixels, uint32_t width, uint32_t height, double anchor_x, double anchor_y, float ui_scale,
                        std::span<const SpriteShape> shapes, SpriteTarget target) {
    for (const SpriteShape& sh : shapes) {
        const ScaledShape sc = scale_shape(sh, ui_scale);
        const int xa = std::max(0, static_cast<int>(std::floor(anchor_x + static_cast<double>(sc.bx0))));
        const int xb = std::min(static_cast<int>(width) - 1, static_cast<int>(std::ceil(anchor_x + static_cast<double>(sc.bx1))));
        const int ya = std::max(0, static_cast<int>(std::floor(anchor_y + static_cast<double>(sc.by0))));
        const int yb = std::min(static_cast<int>(height) - 1, static_cast<int>(std::ceil(anchor_y + static_cast<double>(sc.by1))));
        const int n = sh.aa ? 4 : 1;
        for (int pass = 0; pass < 2; ++pass) {  // the fill, then the stroke over it
            const std::optional<SpriteColour>& colour = pass == 0 ? sh.fill : sh.stroke;
            if (!colour) continue;
            for (int y = ya; y <= yb; ++y) {
                for (int x = xa; x <= xb; ++x) {
                    int hit = 0;
                    for (int j = 0; j < n; ++j) {
                        for (int i = 0; i < n; ++i) {
                            const float fx = static_cast<float>(static_cast<double>(x) + (static_cast<double>(i) + 0.5) / n - anchor_x);
                            const float fy = static_cast<float>(static_cast<double>(y) + (static_cast<double>(j) + 0.5) / n - anchor_y);
                            const P2 p{fx, fy};
                            hit += (pass == 0 ? sc.in_fill(p) : sc.in_stroke(p)) ? 1 : 0;
                        }
                    }
                    if (hit == 0) continue;
                    uint8_t* px = &pixels[(static_cast<size_t>(y) * width + static_cast<size_t>(x)) * 4u];
                    const float cov = static_cast<float>(hit) / static_cast<float>(n * n);
                    if (target == SpriteTarget::overlay_rgba) {
                        blend_overlay(px, *colour, cov);
                    } else {
                        blend(px, *colour, cov);
                    }
                }
            }
        }
    }
}

}  // namespace spade::render
