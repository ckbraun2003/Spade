// ---------------------------------------------------------------------------
// test_render_sprites_s3.cpp -- D182 S3 (KAT's drawing spec, P2 and B-L2, B-L3, B-V2; the lead approved the header, 2026-11-19): world-anchored,
// constant-pixel sprites on the CPU renderer, the reference the GL renderer is held to. A sprite is an anchor in world space and a list of 2D shapes in
// pixels from its projected point; the helpers in render/sprite_kit.hpp build KAT's drone disc, marks and spawn reticle from colours the CALLER passes.
// The geometry is test_render_polyline_s1.cpp's: a 33 x 33 frame, the camera 5 m back on the axis, so the world origin projects to (16.5, 16.5), the
// centre of pixel (16, 16), and 1 m at z = 0 is 5.72 px.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/sprite_kit.hpp"
#include "render/sprites.hpp"

namespace {

using spade::render::Camera;
using spade::render::MarkKind;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;
using spade::render::ReticleState;
using spade::render::SpriteCap;
using spade::render::SpriteColour;
using spade::render::SpriteShape;
using spade::render::SpriteShapeKind;
using spade::render::WorldSprite;

constexpr uint32_t kW = 33, kH = 33;

struct Frame {
    std::vector<uint8_t> px;
    [[nodiscard]] glm::ivec3 rgb(uint32_t x, uint32_t y) const {
        const size_t i = (static_cast<size_t>(y) * kW + x) * 4u;
        return glm::ivec3(px[i + 2], px[i + 1], px[i + 0]);
    }
    [[nodiscard]] bool black(uint32_t x, uint32_t y) const { return rgb(x, y) == glm::ivec3(0, 0, 0); }
    [[nodiscard]] int lit_in_row(uint32_t y) const {
        int n = 0;
        for (uint32_t x = 0; x < kW; ++x) n += black(x, y) ? 0 : 1;
        return n;
    }
    [[nodiscard]] int lit() const {
        int n = 0;
        for (uint32_t y = 0; y < kH; ++y) n += lit_in_row(y);
        return n;
    }
};

[[nodiscard]] RenderScene black_scene() {
    RenderScene scene;
    scene.lighting.sky_zenith = glm::vec3(0.0f);
    scene.lighting.sky_horizon = glm::vec3(0.0f);
    return scene;
}

[[nodiscard]] Frame draw(const RenderScene& scene, RenderOptions options = {}, spade::Result<void>* result = nullptr) {
    Frame f;
    f.px.assign(static_cast<size_t>(kW) * kH * 4u, 0xABu);
    RenderTarget target{.pixels = std::span<uint8_t>(f.px), .width = kW, .height = kH, .stride = kW * 4u};
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    options.overlays = false;
    const spade::Result<void> r = spade::render::render(scene, camera, options, target);
    if (result != nullptr) {
        *result = r;
    } else {
        EXPECT_TRUE(r.has_value()) << r.error().context;
    }
    return f;
}

[[nodiscard]] RenderScene with_sprites(std::vector<WorldSprite> sprites) {
    RenderScene scene = black_scene();
    scene.sprites = std::move(sprites);
    return scene;
}

[[nodiscard]] glm::ivec3 rgb_of(SpriteColour c) { return glm::ivec3(c.r, c.g, c.b); }

constexpr SpriteColour kFill{0xDC, 0xE5, 0xEC, 255};    // KAT's dark OnSurface
constexpr SpriteColour kBorder{0x0B, 0x0F, 0x14, 255};  // dark SurfaceVoid; the pair is >= 200 apart in RGB
constexpr SpriteColour kCrash{0xE2, 0x6E, 0x73, 255};
constexpr SpriteColour kLap{0x5B, 0xBB, 0x86, 255};
constexpr SpriteColour kLimit{0xE8, 0xA3, 0x3D, 255};
constexpr SpriteColour kDim{0x82, 0x8F, 0x9C, 255};
constexpr SpriteColour kAccent{0x7C, 0x93, 0xFF, 255};
constexpr SpriteColour kOk{0x5B, 0xBB, 0x86, 255};

[[nodiscard]] SpriteShape filled_circle(float r, SpriteColour fill) {
    SpriteShape s;
    s.kind = SpriteShapeKind::circle;
    s.r = r;
    s.fill = fill;
    return s;
}

[[nodiscard]] WorldSprite one(SpriteShape shape, glm::dvec3 anchor = glm::dvec3(0.0)) {
    WorldSprite sprite;
    sprite.anchor = anchor;
    sprite.shapes = {std::move(shape)};
    return sprite;
}

}  // namespace

TEST(RenderSpritesS3, TheDefaultsAreKATsAndASceneWithoutSpritesIsByteForByteTheOldFrame) {
    const WorldSprite sprite;
    EXPECT_EQ(sprite.slots, 1u) << "slot 0 only, unlike polylines: overlays stay off policy cameras";
    EXPECT_FALSE(sprite.cull_outside_view);
    EXPECT_FLOAT_EQ(sprite.visible_max_span_px, 0.0f) << "0 = always";
    EXPECT_EQ(spade::render::kMaxSprites, 65536u);
    const RenderScene bare = black_scene();
    EXPECT_TRUE(bare.sprites.empty());
    EXPECT_EQ(draw(bare).lit(), 0);
}

// KAT's droneMarkerIsSmallOneColourWithABorder, ported: a 6 px fill inside a 1.5 px border centred on a circle of radius 3.75, so the fill shows 6 px across
// and the border adds 0.75 px each side; the lit extent across is ceil(6 + 3) = 9, plus or minus one.
TEST(RenderSpritesS3, TheDroneDiscIsSixPixelsOfFillWithABorderAndNothingBeyond) {
    const Frame got = draw(with_sprites({spade::render::make_drone_disc(glm::dvec3(0.0), kFill, kBorder)}));
    EXPECT_EQ(got.rgb(16, 16), rgb_of(kFill));
    EXPECT_EQ(got.rgb(18, 16), rgb_of(kFill)) << "2 px out: still the fill";
    EXPECT_EQ(got.rgb(20, 16), rgb_of(kBorder)) << "4 px out: the border";
    EXPECT_TRUE(got.black(22, 16)) << "6 px out: nothing";
    EXPECT_NEAR(got.lit_in_row(16), 9, 1);
    int column = 0;
    for (uint32_t y = 0; y < kH; ++y) column += got.black(16, y) ? 0 : 1;
    EXPECT_NEAR(column, 9, 1);
}

// The disc is only for a drone that is a speck: it draws iff 2 * hypot(proj(anchor + half_m along the camera's right) - proj(anchor)) < max_span_px,
// and not if either point fails to project. At z = 0, 1 m is 5.72 px, so a half of 1 m is a span of 11.44 px.
TEST(RenderSpritesS3, VisibleIfDrawsOnlyBelowTheSpanAndNeverWhenAPointDoesNotProject) {
    WorldSprite sprite = one(filled_circle(3.0f, kFill));
    sprite.visible_half_m = 1.0f;
    sprite.visible_max_span_px = 12.0f;
    EXPECT_EQ(draw(with_sprites({sprite})).rgb(16, 16), rgb_of(kFill)) << "11.44 < 12";
    sprite.visible_max_span_px = 11.0f;
    EXPECT_EQ(draw(with_sprites({sprite})).lit(), 0) << "11.44 >= 11";
    // the anchor 0.5 m from the camera: the speck rule fails the other way (a big drone's mesh reads), 0.162 m half span is 18.5 px
    const WorldSprite disc = spade::render::make_drone_disc(glm::dvec3(0.0, 0.0, 4.5), kFill, kBorder);
    EXPECT_EQ(draw(with_sprites({disc})).lit(), 0) << "a close drone has no disc";
    // an anchor 1 m ahead of the camera with a generous limit: both points project, so it draws
    WorldSprite edge = one(filled_circle(3.0f, kFill), glm::dvec3(0.0, 0.0, 4.0));
    edge.visible_half_m = 1.0f;
    edge.visible_max_span_px = 1000.0f;
    EXPECT_EQ(draw(with_sprites({edge})).rgb(16, 16), rgb_of(kFill)) << "both points project at 1 m ahead";
    WorldSprite behind = one(filled_circle(3.0f, kFill), glm::dvec3(0.0, 0.0, 6.0));
    EXPECT_EQ(draw(with_sprites({behind})).lit(), 0) << "behind the eye: not projected, nothing drawn";
}

TEST(RenderSpritesS3, NoNearOrFarCullAnAnchorBeyondTheFarPlaneStillDraws) {
    const WorldSprite far_sprite = one(filled_circle(3.0f, kFill), glm::dvec3(0.0, 0.0, -50000.0));
    EXPECT_EQ(draw(with_sprites({far_sprite})).rgb(16, 16), rgb_of(kFill));
}

// KAT: no test pins the marks' painted pixels today, so Spade's tests add them. A 12 px box (6 px each side), a 2 px pen, three colours.
TEST(RenderSpritesS3, TheCrashXIsTwoDiagonalsInTheCrashColour) {
    const Frame got = draw(with_sprites({spade::render::make_mark(MarkKind::crash, glm::dvec3(0.0), kCrash)}));
    EXPECT_EQ(got.rgb(16, 16), rgb_of(kCrash)) << "the lines cross";
    EXPECT_EQ(got.rgb(11, 11), rgb_of(kCrash)) << "5 px up-left on the diagonal";
    EXPECT_EQ(got.rgb(21, 11), rgb_of(kCrash)) << "5 px up-right on the other";
    EXPECT_TRUE(got.black(16, 11)) << "straight up, off both diagonals";
    EXPECT_TRUE(got.black(11, 16));
    EXPECT_TRUE(got.black(24, 24)) << "outside the box and its square caps";
}

TEST(RenderSpritesS3, TheLapRingIsAnOutlineOfDiameterTwelveAndTheTimeLimitSquareAnOutlineOfTwelve) {
    const Frame ring = draw(with_sprites({spade::render::make_mark(MarkKind::lap, glm::dvec3(0.0), kLap)}));
    EXPECT_EQ(ring.rgb(22, 16), rgb_of(kLap)) << "6 px out: on the ring";
    EXPECT_EQ(ring.rgb(16, 10), rgb_of(kLap));
    EXPECT_TRUE(ring.black(16, 16)) << "no fill";
    EXPECT_TRUE(ring.black(25, 16)) << "9 px out: beyond the pen";
    const Frame square = draw(with_sprites({spade::render::make_mark(MarkKind::time_limit, glm::dvec3(0.0), kLimit)}));
    EXPECT_EQ(square.rgb(22, 16), rgb_of(kLimit));
    EXPECT_EQ(square.rgb(16, 22), rgb_of(kLimit));
    EXPECT_EQ(square.rgb(22, 22), rgb_of(kLimit)) << "the corner";
    EXPECT_TRUE(square.black(16, 16));
    EXPECT_TRUE(square.black(25, 16));
    EXPECT_TRUE(ring.black(22, 22)) << "a ring is not a square: the corner of the box is outside the circle";
    EXPECT_NE(ring.lit(), square.lit());
}

// KAT's spawnMarkerRestStateIsDimReticle, occupiedSpawnDrawsADiscAndTheNewLabel...: hard edges, a circle of radius 9 and four ticks from 13 in to 7; the
// hovered one has a halo of radius 13 (a 0x22 fill and an accent stroke) UNDER it; the occupied one a disc of radius 4 (a 0x44 fill and an ok stroke) over it.
TEST(RenderSpritesS3, TheRestingReticleIsACircleAndFourTicksAndNothingAtTheCentre) {
    const Frame got = draw(with_sprites({spade::render::make_spawn_reticle(glm::dvec3(0.0), ReticleState::normal, kDim, kAccent, kOk)}));
    EXPECT_TRUE(got.black(16, 16)) << "centre unpainted";
    EXPECT_EQ(got.rgb(25, 16), rgb_of(kDim)) << "the circle, 9 px out";
    EXPECT_EQ(got.rgb(16, 7), rgb_of(kDim));
    EXPECT_EQ(got.rgb(24, 16), rgb_of(kDim)) << "a tick starts at 7";
    EXPECT_EQ(got.rgb(28, 16), rgb_of(kDim)) << "and runs to 13";
    EXPECT_TRUE(got.black(28, 17)) << "1.2 px wide: the next row is clear";
    EXPECT_TRUE(got.black(31, 16)) << "past the tick's end";
    EXPECT_TRUE(got.black(19, 19)) << "no halo, no disc";
    // hard edges: every lit pixel is exactly the reticle colour
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            if (!got.black(x, y)) {
                EXPECT_EQ(got.rgb(x, y), rgb_of(kDim)) << x << "," << y;
            }
        }
    }
}

TEST(RenderSpritesS3, TheHoveredReticleHasAHaloUnderItAndTheOccupiedOneADiscOverIt) {
    const Frame hover = draw(with_sprites({spade::render::make_spawn_reticle(glm::dvec3(0.0), ReticleState::hover, kAccent, kAccent, kOk)}));
    // the halo's fill, 0x22 of the accent over black, 3 px out where nothing else is
    const glm::ivec3 want(kAccent.r * 34 / 255, kAccent.g * 34 / 255, kAccent.b * 34 / 255);
    EXPECT_NEAR(hover.rgb(19, 16).r, want.r, 1);
    EXPECT_NEAR(hover.rgb(19, 16).g, want.g, 1);
    EXPECT_NEAR(hover.rgb(19, 16).b, want.b, 1);
    EXPECT_EQ(hover.rgb(25, 16), rgb_of(kAccent)) << "the reticle circle over the halo";
    const Frame occupied = draw(with_sprites({spade::render::make_spawn_reticle(glm::dvec3(0.0), ReticleState::occupied, kDim, kAccent, kOk)}));
    const glm::ivec3 disc(kOk.r * 68 / 255, kOk.g * 68 / 255, kOk.b * 68 / 255);
    EXPECT_NEAR(occupied.rgb(16, 16).g, disc.g, 1) << "the disc's 0x44 fill at the centre";
    EXPECT_EQ(occupied.rgb(20, 16), rgb_of(kOk)) << "its 1 px stroke, 4 px out";
    EXPECT_EQ(occupied.rgb(25, 16), rgb_of(kDim)) << "the reticle circle keeps the reticle colour";
    const Frame rest = draw(with_sprites({spade::render::make_spawn_reticle(glm::dvec3(0.0), ReticleState::normal, kDim, kAccent, kOk)}));
    EXPECT_TRUE(rest.black(19, 16)) << "no halo when not hovered";
}

TEST(RenderSpritesS3, ShapesDrawInTheOrderGivenAndSpritesInSubmissionOrder) {
    SpriteShape big = filled_circle(6.0f, kLap);
    SpriteShape small = filled_circle(3.0f, kCrash);
    WorldSprite a;
    a.shapes = {big, small};
    EXPECT_EQ(draw(with_sprites({a})).rgb(16, 16), rgb_of(kCrash)) << "the later shape is on top";
    WorldSprite b;
    b.shapes = {small, big};
    EXPECT_EQ(draw(with_sprites({b})).rgb(16, 16), rgb_of(kLap));
    WorldSprite first = one(big);
    WorldSprite second = one(small);
    EXPECT_EQ(draw(with_sprites({first, second})).rgb(16, 16), rgb_of(kCrash));
    EXPECT_EQ(draw(with_sprites({second, first})).rgb(16, 16), rgb_of(kLap));
}

TEST(RenderSpritesS3, AHardEdgeCoversAPixelWhenItsCentreIsInsideAndAnAntiAliasedOneBlends) {
    SpriteShape hard = filled_circle(3.0f, kFill);
    hard.aa = false;
    const Frame h = draw(with_sprites({one(hard)}));
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            const float dx = static_cast<float>(x) + 0.5f - 16.5f, dy = static_cast<float>(y) + 0.5f - 16.5f;
            const float d2 = dx * dx + dy * dy;
            if (d2 > 8.9f && d2 < 9.1f) continue;  // on the edge itself: either way
            EXPECT_EQ(h.rgb(x, y) == rgb_of(kFill), d2 < 9.0f) << x << "," << y << " d2=" << d2;
        }
    }
    SpriteShape soft = filled_circle(3.0f, kFill);
    const Frame s = draw(with_sprites({one(soft)}));
    // an anti-aliased edge has in-between pixels; a hard one has none
    int partial = 0;
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            if (!s.black(x, y) && s.rgb(x, y) != rgb_of(kFill)) ++partial;
        }
    }
    EXPECT_GT(partial, 0);
}

TEST(RenderSpritesS3, AColourWithAlphaBlendsOverTheFrameAsStraightSourceOver) {
    SpriteShape half = filled_circle(4.0f, SpriteColour{200, 100, 50, 128});
    half.aa = false;
    const Frame got = draw(with_sprites({one(half)}));
    EXPECT_NEAR(got.rgb(16, 16).r, 100, 1) << "200 * 128/255 over black";
    EXPECT_NEAR(got.rgb(16, 16).g, 50, 1);
    EXPECT_NEAR(got.rgb(16, 16).b, 25, 1);
}

TEST(RenderSpritesS3, ASlotMaskAndUiScaleDecideWhereAndHowBig) {
    const WorldSprite lap = spade::render::make_mark(MarkKind::lap, glm::dvec3(0.0), kLap);
    RenderOptions viewer;
    RenderOptions policy;
    policy.slot = 1u;
    EXPECT_GT(draw(with_sprites({lap}), viewer).lit(), 0) << "slot 0 by default";
    EXPECT_EQ(draw(with_sprites({lap}), policy).lit(), 0) << "a policy camera on slot 1 sees none";
    WorldSprite both = lap;
    both.slots = 0b11u;
    EXPECT_GT(draw(with_sprites({both}), policy).lit(), 0);
    // ui_scale 1.5: the 12 px box is 18, the ring's radius 9 and its pen 3
    RenderOptions scaled;
    scaled.ui_scale = 1.5f;
    const Frame big = draw(with_sprites({lap}), scaled);
    EXPECT_EQ(big.rgb(25, 16), rgb_of(kLap)) << "9 px out is on the ring at 1.5";
    EXPECT_TRUE(draw(with_sprites({lap}), viewer).black(25, 16)) << "and not at 1";
    EXPECT_TRUE(big.black(16, 16));
}

TEST(RenderSpritesS3, CullOutsideViewDropsAnAnchorOffTheFrameButOnlyWhenAsked) {
    // x = 3.41 m projects to screen 36, three pixels right of the frame; a circle of radius 6 reaches back in to column 30
    WorldSprite sprite = one(filled_circle(6.0f, kFill), glm::dvec3(3.41, 0.0, 0.0));
    EXPECT_EQ(draw(with_sprites({sprite})).rgb(31, 16), rgb_of(kFill)) << "an element is clipped to the frame, not dropped, by default";
    sprite.cull_outside_view = true;
    EXPECT_EQ(draw(with_sprites({sprite})).lit(), 0) << "the sim view skips a marker whose anchor is off the frame";
    sprite.anchor = glm::dvec3(2.0, 0.0, 0.0);
    EXPECT_GT(draw(with_sprites({sprite})).lit(), 0) << "an anchor inside is kept";
}

TEST(RenderSpritesS3, TheLineAndPolygonAndArcDrawWithTheirCaps) {
    SpriteShape line;
    line.kind = SpriteShapeKind::line;
    line.x0 = -4.0f;
    line.x1 = 4.0f;
    line.stroke = kCrash;
    line.stroke_px = 2.0f;
    line.cap = SpriteCap::square;
    const Frame square = draw(with_sprites({one(line)}));
    // the ends are at 12.5 and 20.5; a square cap on a 2 px pen runs 1 px past each, a flat one stops at the end
    EXPECT_EQ(square.rgb(20, 16), rgb_of(kCrash)) << "a square cap covers the whole pixel at the end";
    EXPECT_TRUE(square.black(22, 16));
    line.cap = SpriteCap::flat;
    const Frame flat = draw(with_sprites({one(line)}));
    EXPECT_TRUE(flat.black(21, 16));
    EXPECT_GT(flat.rgb(20, 16).r, 0) << "the end pixel is half covered";
    EXPECT_LT(flat.rgb(20, 16).r, kCrash.r - 20);
    EXPECT_EQ(flat.rgb(19, 16), rgb_of(kCrash));
    SpriteShape tri;
    tri.kind = SpriteShapeKind::polygon;
    tri.points = {glm::vec2(-5, 5), glm::vec2(5, 5), glm::vec2(0, -5)};
    tri.fill = kLap;
    const Frame poly = draw(with_sprites({one(tri)}));
    EXPECT_EQ(poly.rgb(16, 19), rgb_of(kLap));
    EXPECT_TRUE(poly.black(11, 13));
    SpriteShape arc;
    arc.kind = SpriteShapeKind::arc;
    arc.r = 6.0f;
    arc.start_deg = 0.0f;
    arc.span_deg = 90.0f;  // 0 = 3 o'clock, positive = counter-clockwise, so the upper right on a y-down screen
    arc.stroke = kLimit;
    arc.stroke_px = 2.0f;
    arc.cap = SpriteCap::flat;
    const Frame a = draw(with_sprites({one(arc)}));
    EXPECT_EQ(a.rgb(20, 12), rgb_of(kLimit)) << "the middle of the upper right quarter";
    EXPECT_TRUE(a.black(12, 12)) << "the upper left is outside it";
    EXPECT_TRUE(a.black(12, 20));
    EXPECT_TRUE(a.black(20, 20));
}

TEST(RenderSpritesS3, ASpriteThatCannotBeDrawnIsRefusedByNameAndDrawsNothing) {
    spade::Result<void> r;
    WorldSprite empty;
    (void)draw(with_sprites({one(filled_circle(3.0f, kFill)), empty}), {}, &r);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("sprites[1]"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("shapes"), std::string::npos) << r.error().context;

    SpriteShape bare;  // neither a fill nor a stroke
    bare.kind = SpriteShapeKind::circle;
    bare.r = 3.0f;
    (void)draw(with_sprites({one(bare)}), {}, &r);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("fill"), std::string::npos) << r.error().context;

    SpriteShape two_points;
    two_points.kind = SpriteShapeKind::polygon;
    two_points.points = {glm::vec2(0, 0), glm::vec2(1, 1)};
    two_points.fill = kFill;
    (void)draw(with_sprites({one(two_points)}), {}, &r);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("polygon"), std::string::npos) << r.error().context;

    SpriteShape negative = filled_circle(-1.0f, kFill);
    (void)draw(with_sprites({one(negative)}), {}, &r);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("shapes[0]"), std::string::npos) << r.error().context;

    WorldSprite nan_anchor = one(filled_circle(3.0f, kFill), glm::dvec3(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
    (void)draw(with_sprites({nan_anchor}), {}, &r);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("anchor"), std::string::npos) << r.error().context;

    WorldSprite many = one(filled_circle(3.0f, kFill));
    many.shapes.assign(17u, filled_circle(3.0f, kFill));
    (void)draw(with_sprites({many}), {}, &r);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("16"), std::string::npos) << r.error().context;

    SpriteShape ten_thousand;
    ten_thousand.kind = SpriteShapeKind::polygon;
    ten_thousand.fill = kFill;
    ten_thousand.points.assign(65u, glm::vec2(1, 1));
    (void)draw(with_sprites({one(ten_thousand)}), {}, &r);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("64"), std::string::npos) << r.error().context;
}

TEST(RenderSpritesS3, MoreThanTheBudgetOfSpritesIsRefusedByNameWithTheCount) {
    std::vector<WorldSprite> many(spade::render::kMaxSprites + 1u, one(filled_circle(1.0f, kFill)));
    spade::Result<void> r;
    (void)draw(with_sprites(std::move(many)), {}, &r);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("65536"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find("sprites"), std::string::npos) << r.error().context;
    EXPECT_NE(r.error().context.find(std::to_string(spade::render::kMaxSprites + 1u)), std::string::npos) << "it names the count";
}

TEST(RenderSpritesS3, ASpriteDrawsOverTheFlowPolylinesAndTheScene) {
    RenderScene scene = black_scene();
    spade::render::FlowPolyline line{{glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)}, {glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)}};
    scene.flow_line_style = spade::render::FlowLineStyle{3.0f, 0.0f, 0.35f};
    scene.flow_polylines = {line};
    scene.sprites = {one(filled_circle(2.0f, kLap))};
    const Frame got = draw(scene);
    EXPECT_EQ(got.rgb(16, 16), rgb_of(kLap)) << "the sprite is last, over the line's capsule";
    EXPECT_EQ(got.rgb(12, 16).r, 255) << "and the line is still there past the sprite";
}

// ---------------------------------------------------------------------------
// Option A (the lead's cost order for S3): each sprite is rasterised into its own tile, GL uploads the packed tiles once and draws one quad a tile, and an
// anti-aliased edge is analytic (coverage = clamp(0.5 - signed distance)) instead of 4 x 4 samples. A hard edge (aa = false) is unchanged.
// ---------------------------------------------------------------------------

namespace {

[[nodiscard]] RenderScene mixed_sprites_scene() {
    RenderScene scene = black_scene();
    const SpriteColour reticle{0x82, 0x8F, 0x9C, 255}, accent{0x7C, 0x93, 0xFF, 255};
    scene.sprites.push_back(spade::render::make_drone_disc(glm::dvec3(-1.2, 0.5, 0.0), kFill, kBorder));
    scene.sprites.push_back(spade::render::make_mark(MarkKind::crash, glm::dvec3(-0.4, 0.5, 0.0), kCrash));
    scene.sprites.push_back(spade::render::make_mark(MarkKind::lap, glm::dvec3(0.4, 0.5, 0.0), kLap));
    scene.sprites.push_back(spade::render::make_mark(MarkKind::time_limit, glm::dvec3(1.2, 0.5, 0.0), kLimit));
    scene.sprites.push_back(spade::render::make_spawn_reticle(glm::dvec3(-0.8, -0.5, 0.0), ReticleState::hover, reticle, accent, kOk));
    scene.sprites.push_back(spade::render::make_spawn_reticle(glm::dvec3(0.0, -0.5, 0.0), ReticleState::occupied, reticle, accent, kOk));
    SpriteShape glass = filled_circle(9.0f, SpriteColour{200, 100, 50, 128});
    glass.aa = false;
    scene.sprites.push_back(one(glass, glm::dvec3(0.8, -0.5, 0.0)));
    SpriteShape wedge;
    wedge.kind = SpriteShapeKind::arc;
    wedge.r = 8.0f;
    wedge.start_deg = 20.0f;
    wedge.span_deg = 200.0f;
    wedge.stroke = kAccent;
    wedge.stroke_px = 2.5f;
    wedge.cap = SpriteCap::round;
    scene.sprites.push_back(one(wedge, glm::dvec3(-1.6, -0.5, 0.0)));
    return scene;
}

// The tiles composited in order, source-over with premultiplied colour, over a black frame (what GL does): a B, G, R, X frame like the CPU's.
[[nodiscard]] std::vector<uint8_t> compose_tiles(const spade::render::SpriteTiles& t) {
    std::vector<uint8_t> frame(static_cast<size_t>(kW) * kH * 4u, 0u);
    for (const spade::render::SpriteTile& tile : t.tiles) {
        for (uint32_t y = 0; y < tile.h; ++y) {
            for (uint32_t x = 0; x < tile.w; ++x) {
                const uint8_t* p = &t.pixels[(static_cast<size_t>(tile.offset) + static_cast<size_t>(y) * tile.w + x) * 4u];  // R, G, B, A premultiplied
                uint8_t* f = &frame[(static_cast<size_t>(tile.y + static_cast<int32_t>(y)) * kW + static_cast<size_t>(tile.x + static_cast<int32_t>(x))) * 4u];
                const float inv = 1.0f - static_cast<float>(p[3]) / 255.0f;
                f[0] = static_cast<uint8_t>(std::min(255.0f, static_cast<float>(p[2]) + static_cast<float>(f[0]) * inv + 0.5f));
                f[1] = static_cast<uint8_t>(std::min(255.0f, static_cast<float>(p[1]) + static_cast<float>(f[1]) * inv + 0.5f));
                f[2] = static_cast<uint8_t>(std::min(255.0f, static_cast<float>(p[0]) + static_cast<float>(f[2]) * inv + 0.5f));
            }
        }
    }
    return frame;
}

[[nodiscard]] Camera tile_camera() {
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    return camera;
}

}  // namespace

TEST(RenderSpritesS3, TilesCompositedOverABlackFrameAreTheDirectDrawToOneLevel) {
    const RenderScene scene = mixed_sprites_scene();
    const Frame direct = draw(scene);
    spade::render::SpriteTiles tiles;
    const size_t n = spade::render::rasterize_sprite_tiles(scene, tile_camera(), RenderOptions{}, kW, kH, tiles);
    EXPECT_EQ(n, scene.sprites.size()) << "one tile for each sprite that draws";
    EXPECT_EQ(tiles.tiles.size(), n);
    const std::vector<uint8_t> composed = compose_tiles(tiles);
    int worst = 0;
    int lit = 0;
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            const size_t i = (static_cast<size_t>(y) * kW + x) * 4u;
            for (size_t c = 0; c < 3u; ++c) worst = std::max(worst, std::abs(static_cast<int>(composed[i + c]) - static_cast<int>(direct.px[i + c])));
            lit += direct.black(x, y) ? 0 : 1;
        }
    }
    EXPECT_GT(lit, 300) << "the sprites reach the frame";
    EXPECT_LE(worst, 1) << "tiles over black differ from drawing on the frame by a level of rounding at most";
}

TEST(RenderSpritesS3, EveryDrawnPixelLiesInsideAnEmittedTileAndATileIsClippedToTheFrame) {
    RenderScene scene = mixed_sprites_scene();
    scene.sprites.push_back(spade::render::make_mark(MarkKind::lap, glm::dvec3(3.3, 0.0, 0.0), kLap));  // partly off the right edge
    const Frame direct = draw(scene);
    spade::render::SpriteTiles tiles;
    (void)spade::render::rasterize_sprite_tiles(scene, tile_camera(), RenderOptions{}, kW, kH, tiles);
    for (const spade::render::SpriteTile& t : tiles.tiles) {
        EXPECT_GE(t.x, 0);
        EXPECT_GE(t.y, 0);
        EXPECT_LE(static_cast<uint32_t>(t.x) + t.w, kW);
        EXPECT_LE(static_cast<uint32_t>(t.y) + t.h, kH);
        EXPECT_GT(t.w, 0u);
        EXPECT_GT(t.h, 0u);
    }
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            if (direct.black(x, y)) continue;
            bool inside = false;
            for (const spade::render::SpriteTile& t : tiles.tiles) {
                inside = inside || (static_cast<int32_t>(x) >= t.x && static_cast<int32_t>(x) < t.x + static_cast<int32_t>(t.w) &&
                                    static_cast<int32_t>(y) >= t.y && static_cast<int32_t>(y) < t.y + static_cast<int32_t>(t.h));
            }
            EXPECT_TRUE(inside) << "pixel " << x << "," << y << " is lit outside every tile";
        }
    }
}

TEST(RenderSpritesS3, TilesAreReusedWithoutGrowingAndRepeatExactly) {
    const RenderScene scene = mixed_sprites_scene();
    spade::render::SpriteTiles tiles;
    (void)spade::render::rasterize_sprite_tiles(scene, tile_camera(), RenderOptions{}, kW, kH, tiles);
    const std::vector<uint8_t> first = tiles.pixels;
    const size_t tile_capacity = tiles.tiles.capacity(), pixel_capacity = tiles.pixels.capacity();
    (void)spade::render::rasterize_sprite_tiles(scene, tile_camera(), RenderOptions{}, kW, kH, tiles);
    EXPECT_EQ(tiles.pixels, first) << "the same scene, the same bytes";
    EXPECT_EQ(tiles.tiles.capacity(), tile_capacity) << "a steady frame allocates nothing";
    EXPECT_EQ(tiles.pixels.capacity(), pixel_capacity);
    // a frame with no sprites after one with sprites: no tiles, and the next frame with sprites is a clean one
    RenderScene bare = black_scene();
    EXPECT_EQ(spade::render::rasterize_sprite_tiles(bare, tile_camera(), RenderOptions{}, kW, kH, tiles), 0u);
    EXPECT_TRUE(tiles.tiles.empty());
    EXPECT_TRUE(tiles.pixels.empty());
    (void)spade::render::rasterize_sprite_tiles(scene, tile_camera(), RenderOptions{}, kW, kH, tiles);
    EXPECT_EQ(tiles.pixels, first);
}

TEST(RenderSpritesS3, TheTileRulesAreTheFrameRulesSlotVisibleIfAndProjection) {
    RenderScene scene = mixed_sprites_scene();
    RenderOptions policy;
    policy.slot = 1u;
    spade::render::SpriteTiles tiles;
    EXPECT_EQ(spade::render::rasterize_sprite_tiles(scene, tile_camera(), policy, kW, kH, tiles), 0u) << "slot 0 only by default";
    scene.sprites.push_back(spade::render::make_drone_disc(glm::dvec3(0.0, 0.0, 4.5), kFill, kBorder));   // a close drone: no disc
    scene.sprites.push_back(one(filled_circle(3.0f, kFill), glm::dvec3(0.0, 0.0, 6.0)));                  // behind the eye
    EXPECT_EQ(spade::render::rasterize_sprite_tiles(scene, tile_camera(), RenderOptions{}, kW, kH, tiles), mixed_sprites_scene().sprites.size());
}

// The analytic edge against the 4 x 4 sampling it replaces: random circles, rects, lines, polygons and arcs, white on black, anchored off a pixel centre. An
// edge pixel may differ (a line's ramp and a corner are not the sampled area); everything away from an edge is the same.
TEST(RenderSpritesS3, TheAnalyticEdgeAgreesWithTheSampledEdgeAwayFromTheEdgeAndWithinABandOnIt) {
    uint32_t state = 12345u;
    const auto rnd = [&state]() {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) / static_cast<float>(1u << 24);
    };
    constexpr uint32_t kN = 48;
    int worst = 0;
    size_t differing = 0, compared = 0;
    for (int k = 0; k < 120; ++k) {
        SpriteShape s;
        s.aa = true;
        const SpriteColour white{255, 255, 255, 255};
        s.stroke_px = 1.0f + 3.0f * rnd();
        switch (k % 5) {
            case 0:
                s.kind = SpriteShapeKind::circle;
                s.r = 3.0f + 14.0f * rnd();
                s.fill = (k % 2 == 0) ? std::optional<SpriteColour>(white) : std::nullopt;
                s.stroke = (k % 2 == 0) ? std::nullopt : std::optional<SpriteColour>(white);
                break;
            case 1:
                s.kind = SpriteShapeKind::rect;
                s.x0 = -12.0f * rnd();
                s.y0 = -12.0f * rnd();
                s.x1 = 4.0f + 14.0f * rnd();
                s.y1 = 4.0f + 14.0f * rnd();
                s.fill = (k % 2 == 0) ? std::optional<SpriteColour>(white) : std::nullopt;
                s.stroke = (k % 2 == 0) ? std::nullopt : std::optional<SpriteColour>(white);
                break;
            case 2:
                s.kind = SpriteShapeKind::line;
                s.x0 = -14.0f * rnd();
                s.y0 = -14.0f * rnd();
                s.x1 = 14.0f * rnd();
                s.y1 = 14.0f * rnd();
                s.stroke = white;
                s.cap = (k % 3 == 0) ? SpriteCap::square : ((k % 3 == 1) ? SpriteCap::flat : SpriteCap::round);
                break;
            case 3:
                s.kind = SpriteShapeKind::polygon;
                s.points = {glm::vec2(-14.0f * rnd(), -14.0f * rnd()), glm::vec2(14.0f * rnd(), -12.0f * rnd()), glm::vec2(12.0f * rnd(), 14.0f * rnd()),
                            glm::vec2(-12.0f * rnd(), 12.0f * rnd())};
                s.fill = (k % 2 == 0) ? std::optional<SpriteColour>(white) : std::nullopt;
                s.stroke = (k % 2 == 0) ? std::nullopt : std::optional<SpriteColour>(white);
                break;
            default:
                s.kind = SpriteShapeKind::arc;
                s.r = 5.0f + 12.0f * rnd();
                s.start_deg = 360.0f * rnd();
                s.span_deg = 40.0f + 280.0f * rnd();
                s.stroke = white;
                s.cap = (k % 3 == 0) ? SpriteCap::square : ((k % 3 == 1) ? SpriteCap::flat : SpriteCap::round);
                break;
        }
        std::vector<uint8_t> a(static_cast<size_t>(kN) * kN * 4u, 0u), b = a;
        const std::vector<SpriteShape> shapes = {s};
        spade::render::draw_sprite_shapes(a, kN, kN, 24.3, 23.7, 1.0f, shapes);
        spade::render::draw_sprite_shapes_sampled(b, kN, kN, 24.3, 23.7, 1.0f, shapes);
        for (size_t i = 0; i < a.size(); i += 4u) {
            const int d = std::abs(static_cast<int>(a[i + 2]) - static_cast<int>(b[i + 2]));
            worst = std::max(worst, d);
            differing += d != 0 ? 1u : 0u;
            ++compared;
        }
    }
    std::printf("[ S3 analytic edge ] vs the 4x4 sampled edge over 120 random shapes: worst %d levels; %zu of %zu pixels differ\n", worst, differing, compared);
    constexpr int kBandWorstLevels = 255;       // to be pinned at the measured value + 1 once measured
    EXPECT_LE(worst, kBandWorstLevels);
    EXPECT_LT(differing, compared / 10u) << "only edge pixels differ";
}
