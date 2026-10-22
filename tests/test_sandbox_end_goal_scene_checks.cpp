// The end-goal smoke's checks, batch 2 (N20, docs/design/test-docs/
// 2026-10-19-end-goal-checks.md §1, §2, §3, §5, §6, §7, §7a, §8): the
// library, the identities a copy keeps, an import's name, the region's
// bookkeeping, the views, save and reopen, and the measuring helpers for a
// placed object. Each check fails on a stub of the fault it exists to catch
// and passes on a stub of the right state.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "../sandbox/end_goal_checks.hpp"
#include "core/flow_volume.hpp"
#include "render/csg_mesh.hpp"
#include "render/scene.hpp"
#include "render/view_mode.hpp"
#include "world/sdf.hpp"

namespace eg = spade::sandbox::end_goal;

namespace {

[[nodiscard]] bool has(const std::string& s, const std::string& part) { return s.find(part) != std::string::npos; }

[[nodiscard]] bool judged_fail(const eg::Verdict& v) {
    return !v.ok && !has(v.detail, "absent:") && !has(v.detail, "mis-staged:") && !has(v.detail, "under-resolved:");
}

// ---------------------------------------------------------------------------
// Images
// ---------------------------------------------------------------------------

constexpr eg::Rgb kClear{20, 24, 30};

struct Image {
    uint32_t w = 0, h = 0;
    std::vector<uint8_t> rgba;
    [[nodiscard]] eg::PixelsView view() const { return eg::pixels(w, h, rgba); }
    void set(uint32_t x, uint32_t y, eg::Rgb c) {
        uint8_t* p = rgba.data() + (std::size_t{y} * w + x) * 4u;
        p[0] = c[0];
        p[1] = c[1];
        p[2] = c[2];
        p[3] = 255;
    }
};

// `w` x `h` in `fill`, with the rectangle [x0, x1) x [y0, y1) in `ink`.
[[nodiscard]] Image image(uint32_t w, uint32_t h, eg::Rgb fill, eg::Rgb ink = {}, uint32_t x0 = 0, uint32_t y0 = 0,
                          uint32_t x1 = 0, uint32_t y1 = 0) {
    Image img{w, h, std::vector<uint8_t>(std::size_t{w} * h * 4u)};
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const bool in = x >= x0 && x < x1 && y >= y0 && y < y1;
            img.set(x, y, in ? ink : fill);
        }
    }
    return img;
}

// 64 x 48 on the clear colour, with a 16 x 19 drawn rectangle: 304 pixels, 9.9%.
[[nodiscard]] Image drawn(eg::Rgb ink, uint32_t x0 = 10) { return image(64, 48, kClear, ink, x0, 10, x0 + 16, 29); }

// A thumbnail: a gradient, so no colour dominates.
[[nodiscard]] Image gradient(uint8_t tint) {
    Image img = image(64, 48, {0, 0, 0});
    for (uint32_t y = 0; y < 48; ++y) {
        for (uint32_t x = 0; x < 64; ++x) {
            img.set(x, y, {static_cast<uint8_t>(4 * x), static_cast<uint8_t>(5 * y), tint});
        }
    }
    return img;
}

// A grey placeholder framed by a one-pixel border.
[[nodiscard]] Image framed_placeholder() {
    Image img = image(64, 48, {128, 128, 128});
    for (uint32_t x = 0; x < 64; ++x) {
        img.set(x, 0, {255, 255, 255});
        img.set(x, 47, {255, 255, 255});
    }
    for (uint32_t y = 0; y < 48; ++y) {
        img.set(0, y, {255, 255, 255});
        img.set(63, y, {255, 255, 255});
    }
    return img;
}

[[nodiscard]] std::vector<eg::PixelsView> views_of(const std::vector<Image>& images) {
    std::vector<eg::PixelsView> v;
    for (const Image& img : images) v.push_back(img.view());
    return v;
}

[[nodiscard]] std::vector<Image> nine_thumbnails() {
    std::vector<Image> images;
    for (int i = 0; i < 9; ++i) images.push_back(gradient(static_cast<uint8_t>(25 * i)));
    return images;
}

// A `w` x `h` view's background with the items hidden: the clear colour.
// Kept, since the views point into it.
[[nodiscard]] const Image& clear_background(uint32_t w, uint32_t h) {
    static std::deque<Image> kept;
    for (const Image& img : kept) {
        if (img.w == w && img.h == h) return img;
    }
    return kept.emplace_back(image(w, h, kClear));
}

// A mode switched to: supported, applied, drawn, honoured, with `frame`,
// over the clear background, and still (grabbed again unchanged).
[[nodiscard]] eg::ModeShown mode(const char* kind, const Image& frame) {
    eg::ModeShown m;
    m.kind = kind;
    m.supported = m.applied = m.drawn = true;
    m.frame = frame.view();
    m.background = clear_background(frame.w, frame.h).view();
    m.again = frame.view();
    return m;
}

// A sky: a vertical gradient, no one colour, as the editor's view draws.
[[nodiscard]] Image sky(uint32_t w = 64, uint32_t h = 48) {
    Image img = image(w, h, {0, 0, 0});
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) img.set(x, y, {static_cast<uint8_t>(90 + 3 * y), 150, 230});
    }
    return img;
}

// ---------------------------------------------------------------------------
// Library, identities, import, region
// ---------------------------------------------------------------------------

[[nodiscard]] std::vector<eg::LibraryEntry> index_of_nine() {
    return {{"reference quad", "drones", "drones/ref_quad.model.yaml"},
            {"box", "primitives", "objects/box.object.yaml"},
            {"sphere", "primitives", "objects/sphere.object.yaml"},
            {"cylinder", "primitives", "objects/cylinder.object.yaml"},
            {"capsule", "primitives", "objects/capsule.object.yaml"},
            {"gate", "obstacles", "objects/gate.object.yaml"},
            {"floor tile", "floors", "objects/floor_tile.object.yaml"},
            {"pole", "obstacles", "objects/pole.object.yaml"},
            {"wall", "obstacles", "objects/wall.object.yaml"}};
}

[[nodiscard]] std::vector<eg::Identity> drone_ids() {
    return {{"model", 0xA1u, 0xA1u}, {"parts", 0xB2u, 0xB2u}, {"propulsion", 0xC3u, 0xC3u}};
}

[[nodiscard]] eg::ImportShown good_import() {
    eg::ImportShown m;
    m.shown_name = "bench";
    m.mesh_name = "bench";
    m.file = "imports/park_bench.gltf";
    m.shown_source = "imports/park_bench.gltf";
    m.sidecar_loaded = true;
    m.gltf_vertices = m.sidecar_vertices = 1200;
    m.gltf_triangles = m.sidecar_triangles = 2000;
    return m;
}

[[nodiscard]] spade::render::ViewLegend speed_legend(float max, bool from_header = true) {
    spade::render::ViewLegend l;
    l.layer = spade::render::FlowView::streamlines;
    l.quantity = "speed |u|";
    l.unit = "m/s";
    l.range.min = 0.0f;
    l.range.max = max;
    l.range.from_header = from_header;
    return l;
}

[[nodiscard]] eg::LegendCheck legends(std::vector<spade::render::ViewLegend> returned, bool running = true) {
    eg::LegendCheck c;
    c.layers = returned.size();
    c.returned = returned;
    c.shown = std::move(returned);
    c.region_running = running;
    return c;
}

// ---------------------------------------------------------------------------
// SDF programs, postfix
// ---------------------------------------------------------------------------

[[nodiscard]] glm::vec4 params_of(spade::SdfPrim kind) {
    switch (kind) {
        case spade::SdfPrim::plane: return {0.0f, 1.0f, 0.0f, 0.0f};
        case spade::SdfPrim::sphere: return {0.5f, 0.0f, 0.0f, 0.0f};
        case spade::SdfPrim::box: return {0.5f, 0.4f, 0.3f, 0.0f};
        case spade::SdfPrim::cylinder: return {0.2f, 0.5f, 0.0f, 0.0f};
        case spade::SdfPrim::capsule: return {0.2f, 0.4f, 0.0f, 0.0f};
        case spade::SdfPrim::torus: return {0.6f, 0.1f, 0.0f, 0.0f};
        default: return {};
    }
}

[[nodiscard]] spade::SdfNode leaf(spade::SdfPrim kind) {
    spade::SdfNode n;
    n.kind = static_cast<uint32_t>(kind);
    n.op = static_cast<uint32_t>(spade::SdfOp::none);
    n.params = params_of(kind);
    return n;
}

[[nodiscard]] spade::SdfNode op(spade::SdfOp o) {
    spade::SdfNode n;
    n.op = static_cast<uint32_t>(o);
    return n;
}

[[nodiscard]] spade::SdfProgram program(std::vector<spade::SdfNode> nodes) {
    spade::SdfProgram p;
    p.nodes = std::move(nodes);
    p.transforms.push_back(spade::SdfTransform{});
    p.node_materials.assign(p.nodes.size(), 0u);
    return p;
}

// The ground: the plane y = 0, solid below.
[[nodiscard]] spade::SdfProgram ground() { return program({leaf(spade::SdfPrim::plane)}); }

// A 1 m box drawable whose bottom face sits at `bottom_y`.
[[nodiscard]] spade::render::RenderScene box_scene(float bottom_y) {
    spade::render::RenderScene scene;
    spade::render::MeshData mesh;
    for (const float x : {-0.5f, 0.5f}) {
        for (const float y : {-0.5f, 0.5f}) {
            for (const float z : {-0.5f, 0.5f}) mesh.positions.emplace_back(x, y, z);
        }
    }
    scene.meshes.push_back(mesh);
    spade::render::DrawItem item;
    item.mesh_index = 0;
    item.local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(2.0f, bottom_y + 0.5f, -1.0f));
    scene.statics.push_back(item);
    return scene;
}

[[nodiscard]] eg::RoundTrip good_round_trip() {
    eg::RoundTrip t;
    t.saved = "scene_version: 4\nairflow:\n  - kind: follow\n";
    t.reemitted = t.saved;
    t.drones = 1;
    t.identities = {{"scene", 0x1111u, 0x1111u},
                    {"region", 0x2222u, 0x2222u},
                    {"drone 0 model", 0x3333u, 0x3333u},
                    {"drone 0 parts", 0x4444u, 0x4444u},
                    {"drone 0 propulsion", 0x5555u, 0x5555u}};
    t.steps_before = t.steps_after = eg::kRoundTripSteps;
    spade::BodyState b{};
    b.pos = glm::vec3(0.1f, 1.25f, -0.3f);
    b.vel = glm::vec3(0.0f, 0.42f, 0.0f);
    b.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    b.mass = 0.526f;
    b.inv_inertia_diag = glm::vec3(250.0f, 180.0f, 250.0f);
    t.bodies_before = {b, b};
    t.bodies_after = t.bodies_before;
    return t;
}

}  // namespace

// ===========================================================================
// Measuring helpers (§7a)
// ===========================================================================

TEST(SandboxEndGoalMeasure, CountsTheDrawablesAColliderDeclares) {
    using spade::SdfOp;
    using spade::SdfPrim;
    EXPECT_EQ(eg::declared_drawables(program({leaf(SdfPrim::box)})), 1u);
    // A gate: a torus and two posts, under union only.
    EXPECT_EQ(eg::declared_drawables(program({leaf(SdfPrim::torus), leaf(SdfPrim::cylinder), op(SdfOp::union_),
                                              leaf(SdfPrim::cylinder), op(SdfOp::union_)})),
              3u);
    // A box with a hole: one CSG root, whatever it holds.
    EXPECT_EQ(eg::declared_drawables(program({leaf(SdfPrim::box), leaf(SdfPrim::sphere), op(SdfOp::subtract)})), 1u);
    EXPECT_EQ(eg::declared_drawables(program({leaf(SdfPrim::box), leaf(SdfPrim::sphere), leaf(SdfPrim::capsule),
                                              op(SdfOp::union_), op(SdfOp::intersect)})),
              1u);
    // A sphere beside a holed box: two.
    EXPECT_EQ(eg::declared_drawables(program({leaf(SdfPrim::sphere), leaf(SdfPrim::box), leaf(SdfPrim::sphere),
                                              op(SdfOp::smooth_union), op(SdfOp::union_)})),
              2u);
    // Two CSG roots under a union: two.
    EXPECT_EQ(eg::declared_drawables(program({leaf(SdfPrim::box), leaf(SdfPrim::sphere), op(SdfOp::subtract),
                                              leaf(SdfPrim::cylinder), leaf(SdfPrim::capsule),
                                              op(SdfOp::intersect), op(SdfOp::union_)})),
              2u);
}

TEST(SandboxEndGoalMeasure, AMalformedColliderDeclaresNothing) {
    using spade::SdfOp;
    using spade::SdfPrim;
    EXPECT_EQ(eg::declared_drawables(program({op(SdfOp::union_)})), 0u);
    EXPECT_EQ(eg::declared_drawables(program({leaf(SdfPrim::box), leaf(SdfPrim::box)})), 0u);
    EXPECT_EQ(eg::declared_drawables(program({})), 0u);
}

// A test-only agreement check: the predicate counts on its own (TD-4), and
// here it agrees with the renderer's split_program() over the library's
// shapes and their compositions.
TEST(SandboxEndGoalMeasure, AgreesWithTheRenderersSplitOverTheLibrarysShapes) {
    using spade::SdfOp;
    using spade::SdfPrim;
    const std::vector<std::vector<spade::SdfNode>> colliders = {
        {leaf(SdfPrim::box)},
        {leaf(SdfPrim::sphere)},
        {leaf(SdfPrim::cylinder)},
        {leaf(SdfPrim::capsule)},
        {leaf(SdfPrim::torus), leaf(SdfPrim::cylinder), op(SdfOp::union_), leaf(SdfPrim::cylinder), op(SdfOp::union_)},
        {leaf(SdfPrim::box), leaf(SdfPrim::sphere), op(SdfOp::subtract)},
        {leaf(SdfPrim::sphere), leaf(SdfPrim::box), leaf(SdfPrim::sphere), op(SdfOp::smooth_union), op(SdfOp::union_)},
        {leaf(SdfPrim::box), leaf(SdfPrim::sphere), op(SdfOp::subtract), leaf(SdfPrim::cylinder),
         leaf(SdfPrim::capsule), op(SdfOp::intersect), op(SdfOp::union_)},
    };
    for (std::size_t i = 0; i < colliders.size(); ++i) {
        const spade::SdfProgram p = program(colliders[i]);
        ASSERT_TRUE(p.validate().has_value()) << "collider " << i << ": " << p.validate().error().context;
        const auto split = spade::render::split_program(p);
        ASSERT_TRUE(split.has_value()) << "collider " << i;
        EXPECT_EQ(eg::declared_drawables(p), split->union_primitive_nodes.size() + split->csg_roots.size())
            << "collider " << i;
    }
}

TEST(SandboxEndGoalMeasure, PosesEveryVertexOfTheDrawItems) {
    const spade::render::RenderScene scene = box_scene(0.0f);
    const auto points = eg::posed_vertices(scene, scene.statics);
    ASSERT_EQ(points.size(), 8u);
    EXPECT_FLOAT_EQ(points.front().x, 1.5f);
    EXPECT_FLOAT_EQ(points.front().y, 0.0f);
    EXPECT_FLOAT_EQ(points.front().z, -1.5f);
    EXPECT_FLOAT_EQ(points.back().y, 1.0f);
}

TEST(SandboxEndGoalMeasure, AnItemWithoutItsMeshPosesNothing) {
    spade::render::RenderScene scene = box_scene(0.0f);
    spade::render::DrawItem missing = scene.statics.front();
    missing.mesh_index = 7;
    scene.statics.push_back(missing);
    EXPECT_TRUE(eg::posed_vertices(scene, scene.statics).empty());
}

TEST(SandboxEndGoalMeasure, TheRestGapIsTheLeastDistanceToTheWorldWithoutTheObject) {
    const spade::SdfProgram world = ground();
    for (const float bottom : {0.0f, 0.004f, -0.01f}) {
        const spade::render::RenderScene scene = box_scene(bottom);
        const auto points = eg::posed_vertices(scene, scene.statics);
        EXPECT_NEAR(eg::rest_gap(world, points), bottom, 1e-6f) << bottom;
    }
    EXPECT_TRUE(std::isnan(eg::rest_gap(world, {})));
}

// A box collider (half extents 0.5, 0.4, 0.3) standing `bottom` above the
// ground, and a mesh like its surface nets: a grid over its bottom face and its
// top face's corners, each `inset` inside the surface.
struct RestCase {
    spade::SdfProgram collider = program({leaf(spade::SdfPrim::box)});
    glm::mat4 pose{1.0f};
    std::vector<glm::vec3> vertices;
};

[[nodiscard]] RestCase box_rest_case(float bottom, float inset) {
    RestCase c;
    c.pose[3] = glm::vec4(0.0f, bottom + 0.4f, 0.0f, 1.0f);
    for (const float x : {-0.4f, -0.2f, 0.0f, 0.2f, 0.4f}) {
        for (const float z : {-0.2f, 0.0f, 0.2f}) c.vertices.emplace_back(x, bottom + inset, z);
    }
    for (const float x : {-0.4f, 0.4f}) {
        for (const float z : {-0.2f, 0.2f}) c.vertices.emplace_back(x, bottom + 0.8f - inset, z);
    }
    return c;
}

// The bias the march removes: a mesh 6 mm inside reads 6 mm high, past the
// +5 mm band, while the collider marched under its lowest vertices reads the
// true 3 mm.
TEST(SandboxEndGoalMeasure, AnSdfObjectRestsOnItsCollidersLowestPoints) {
    const RestCase c = box_rest_case(0.003f, 0.006f);
    EXPECT_NEAR(eg::rest_gap(ground(), c.vertices), 0.009f, 1e-6f);
    const std::vector<glm::vec3> points = eg::collider_rest_points(c.collider, c.pose, c.vertices);
    ASSERT_EQ(points.size(), 15u);  // the bottom grid; the top corners are past the band
    for (const glm::vec3& p : points) {
        EXPECT_LE(p.y, 0.003f + 1e-6f) << p.x << ", " << p.z;
        EXPECT_GT(p.y, 0.003f - eg::kRestMarchTolerance) << p.x << ", " << p.z;
    }
    EXPECT_NEAR(eg::rest_gap(ground(), points), 0.003f, eg::kRestMarchTolerance);
}

// Posed by a yaw of 90 degrees and a scale of 2, the box is 1.2 by 1.6 by 2.0
// and buried 1 mm.
TEST(SandboxEndGoalMeasure, TheCollidersPoseTurnsAndScalesIt) {
    RestCase c;
    c.pose = glm::mat4(glm::vec4(0.0f, 0.0f, -2.0f, 0.0f), glm::vec4(0.0f, 2.0f, 0.0f, 0.0f),
                       glm::vec4(2.0f, 0.0f, 0.0f, 0.0f), glm::vec4(1.0f, 0.799f, -2.0f, 1.0f));
    for (const float x : {-0.5f, 0.0f, 0.5f}) {
        for (const float z : {-0.9f, 0.0f, 0.9f}) c.vertices.emplace_back(1.0f + x, 0.003f, -2.0f + z);
    }
    const std::vector<glm::vec3> points = eg::collider_rest_points(c.collider, c.pose, c.vertices);
    ASSERT_EQ(points.size(), 9u);
    EXPECT_NEAR(eg::rest_gap(ground(), points), -0.001f, eg::kRestMarchTolerance);
}

// A column beside the object meets nothing and gives no point; a vertex set
// whose lowest column starts inside the object can't be the object's.
TEST(SandboxEndGoalMeasure, AColumnBesideTheObjectGivesNoPoint) {
    RestCase c = box_rest_case(0.0f, 0.004f);
    c.vertices.emplace_back(2.0f, 0.004f, 0.0f);
    EXPECT_EQ(eg::collider_rest_points(c.collider, c.pose, c.vertices).size(), 15u);
    const std::vector<glm::vec3> beside{{2.0f, 0.004f, 0.0f}};
    EXPECT_TRUE(eg::collider_rest_points(c.collider, c.pose, beside).empty());
    const std::vector<glm::vec3> top_only{{0.0f, 0.796f, 0.0f}};  // 0.25 m under it is inside the box
    EXPECT_TRUE(eg::collider_rest_points(c.collider, c.pose, top_only).empty());
}

TEST(SandboxEndGoalMeasure, NoRestPointsFromBadInputs) {
    const RestCase c = box_rest_case(0.0f, 0.004f);
    EXPECT_TRUE(eg::collider_rest_points(c.collider, c.pose, {}).empty());
    EXPECT_TRUE(eg::collider_rest_points(spade::SdfProgram{}, c.pose, c.vertices).empty());
    std::vector<glm::vec3> nan_vertex = c.vertices;
    nan_vertex[2].x = std::numeric_limits<float>::quiet_NaN();
    EXPECT_TRUE(eg::collider_rest_points(c.collider, c.pose, nan_vertex).empty());
    glm::mat4 stretched = c.pose;
    stretched[0] *= 2.0f;  // not a uniform scale
    EXPECT_TRUE(eg::collider_rest_points(c.collider, stretched, c.vertices).empty());
}

TEST(SandboxEndGoalMeasure, ANonFinitePointMakesTheRestGapNaN) {
    const spade::SdfProgram world = ground();
    const spade::render::RenderScene scene = box_scene(0.0f);
    std::vector<glm::vec3> points = eg::posed_vertices(scene, scene.statics);
    points[3].y = std::numeric_limits<float>::quiet_NaN();
    EXPECT_TRUE(std::isnan(eg::rest_gap(world, points)));
}

// ===========================================================================
// Images
// ===========================================================================

TEST(SandboxEndGoalImages, DiffersCountsPixelsPastEightLevels) {
    const Image base = image(10, 10, {100, 100, 100});
    const Image by8 = image(10, 10, {100, 100, 100}, {108, 100, 100}, 0, 0, 10, 10);
    const Image by9 = image(10, 10, {100, 100, 100}, {100, 91, 100}, 0, 0, 10, 10);
    EXPECT_EQ(eg::differ_fraction(base.view(), by8.view()), 0.0);
    EXPECT_EQ(eg::differ_fraction(base.view(), by9.view()), 1.0);
    const Image one = drawn({200, 180, 40});
    EXPECT_NEAR(eg::differ_fraction(one.view(), kClear), 304.0 / (64.0 * 48.0), 1e-12);
}

TEST(SandboxEndGoalImages, DiffersAtExactlyOnePercent) {
    // 100 of 10,000 pixels differ: 1%, differs. 99: does not.
    const Image hundred = image(100, 100, {50, 50, 50}, {200, 50, 50}, 0, 0, 100, 1);
    const Image ninety_nine = image(100, 100, {50, 50, 50}, {200, 50, 50}, 0, 0, 99, 1);
    EXPECT_TRUE(eg::differs(eg::differ_fraction(hundred.view(), eg::Rgb{50, 50, 50})));
    EXPECT_FALSE(eg::differs(eg::differ_fraction(ninety_nine.view(), eg::Rgb{50, 50, 50})));
}

TEST(SandboxEndGoalImages, ARegionOfAFrameIsItsHalf) {
    const Image frame = image(64, 48, kClear, {200, 30, 30}, 32, 0, 64, 48);
    const eg::PixelsView left = eg::region_of(frame.view(), 0, 0, 32, 48);
    const eg::PixelsView right = eg::region_of(frame.view(), 32, 0, 32, 48);
    EXPECT_EQ(eg::differ_fraction(left, kClear), 0.0);
    EXPECT_EQ(eg::differ_fraction(right, kClear), 1.0);
}

TEST(SandboxEndGoalImages, ARegionOffsetDownAndClippedToTheFrame) {
    // The bottom 8 rows drawn; a region from row 40, asking for 20 rows, gets 8.
    const Image frame = image(64, 48, kClear, {200, 30, 30}, 0, 40, 64, 48);
    const eg::PixelsView bottom = eg::region_of(frame.view(), 16, 40, 100, 20);
    EXPECT_EQ(bottom.width, 48u);
    EXPECT_EQ(bottom.height, 8u);
    EXPECT_EQ(eg::differ_fraction(bottom, kClear), 1.0);
    const eg::PixelsView above = eg::region_of(frame.view(), 0, 20, 64, 20);
    EXPECT_EQ(eg::differ_fraction(above, kClear), 0.0);
    EXPECT_FALSE(eg::well_formed(eg::region_of(frame.view(), 64, 0, 4, 4)));
}

TEST(SandboxEndGoalImages, NamesASpecByItsBaseAndLayers) {
    spade::render::ViewSpec spec;
    spec.base = spade::render::ViewBase::wireframe;
    spec.layers.resize(2);
    spec.layers[0].kind = spade::render::FlowView::vortex_iso;
    spec.layers[1].kind = spade::render::FlowView::streamlines;
    EXPECT_EQ(eg::spec_name(spec), "wireframe+vortex_iso+streamlines");
    EXPECT_EQ(eg::spec_name(spade::render::ViewSpec{}), "shaded");
}

// ===========================================================================
// editor.library
// ===========================================================================

TEST(SandboxEndGoalLibrary, PassesThePanelListingTheIndexInOrder) {
    const auto index = index_of_nine();
    EXPECT_TRUE(eg::lists_index(index, index).ok);
}

TEST(SandboxEndGoalLibrary, FailsADroppedOrReorderedOrRenamedEntry) {
    const auto index = index_of_nine();
    auto dropped = index;
    dropped.pop_back();
    auto reordered = index;
    std::swap(reordered[2], reordered[3]);
    auto other_file = index;
    other_file[5].file = "objects/gate_v2.object.yaml";
    for (const auto* panel : {&dropped, &reordered, &other_file}) {
        const eg::Verdict v = eg::lists_index(index, *panel);
        EXPECT_TRUE(judged_fail(v)) << v.detail;
    }
}

TEST(SandboxEndGoalLibrary, FailsAnIndexShortOfTheFirstEntries) {
    auto index = index_of_nine();
    index.pop_back();
    const eg::Verdict v = eg::lists_index(index, index);
    EXPECT_TRUE(judged_fail(v)) << v.detail;
}

TEST(SandboxEndGoalLibrary, PassesEachThumbnailShowingItsOwnFile) {
    const std::vector<Image> files = nine_thumbnails();
    const std::vector<Image> shown = files;
    const eg::Verdict v = eg::thumbnails(views_of(shown), views_of(files));
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalLibrary, FailsAPlaceholderOrAnotherEntrysImageShown) {
    const std::vector<Image> files = nine_thumbnails();
    std::vector<Image> framed = files;
    framed[4] = framed_placeholder();  // grey, with a one-pixel border: not one colour by its first pixel
    std::vector<Image> swapped = files;
    swapped[4] = files[5];  // another entry's image
    std::vector<Image> blank = files;
    blank[4] = image(64, 48, {128, 128, 128});
    for (const std::vector<Image>* shown : {&framed, &swapped, &blank}) {
        const eg::Verdict v = eg::thumbnails(views_of(*shown), views_of(files));
        EXPECT_TRUE(judged_fail(v)) << v.detail;
    }
}

TEST(SandboxEndGoalLibrary, FailsAFileThatIsItselfOneColour) {
    std::vector<Image> files = nine_thumbnails();
    files[2] = image(64, 48, {128, 128, 128}, {255, 255, 255}, 0, 0, 64, 1);  // 64 of 3,072 pixels off: 2.1%
    files[3] = image(64, 48, {128, 128, 128}, {255, 255, 255}, 0, 0, 30, 1);  // 30 of 3,072: 0.98%, one colour
    const std::vector<Image> shown = files;
    const eg::Verdict v = eg::thumbnails(views_of(shown), views_of(files));
    EXPECT_TRUE(judged_fail(v)) << v.detail;
    EXPECT_TRUE(has(v.detail, "thumbnail 3")) << v.detail;
}

TEST(SandboxEndGoalLibrary, FailsAMissingOrMissizedThumbnail) {
    const std::vector<Image> files = nine_thumbnails();
    std::vector<eg::PixelsView> shown = views_of(files);
    shown[4] = eg::PixelsView{};
    const eg::Verdict missing = eg::thumbnails(shown, views_of(files));
    EXPECT_TRUE(judged_fail(missing)) << missing.detail;
    const Image small = image(32, 24, {10, 200, 10});
    shown = views_of(files);
    shown[4] = small.view();
    const eg::Verdict missized = eg::thumbnails(shown, views_of(files));
    EXPECT_TRUE(judged_fail(missized)) << missized.detail;
    shown = views_of(files);
    shown.pop_back();
    const eg::Verdict short_by_one = eg::thumbnails(shown, views_of(files));
    EXPECT_TRUE(judged_fail(short_by_one)) << short_by_one.detail;
}

// ===========================================================================
// propulsion-kept (and every identity a copy keeps)
// ===========================================================================

TEST(SandboxEndGoalIdentities, PassesEveryRequiredIdentityKept) {
    const eg::Verdict v = eg::identities_kept("propulsion-kept", drone_ids(), eg::kDroneIdentities);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalIdentities, FailsAChainLostInTheCopyAndNamesIt) {
    auto ids = drone_ids();
    ids[2].got = 0u;
    const eg::Verdict v = eg::identities_kept("propulsion-kept", ids, eg::kDroneIdentities);
    EXPECT_TRUE(judged_fail(v)) << v.detail;
    EXPECT_TRUE(has(v.detail, "propulsion")) << v.detail;
}

TEST(SandboxEndGoalIdentities, IsAbsentWhenARequiredIdentityIsLeftOutOrUnset) {
    auto left_out = drone_ids();
    left_out.pop_back();  // no propulsion given
    const eg::Verdict a = eg::identities_kept("propulsion-kept", left_out, eg::kDroneIdentities);
    EXPECT_FALSE(a.ok) << a.detail;
    EXPECT_TRUE(has(a.detail, "absent:") && has(a.detail, "propulsion")) << a.detail;
    auto unset = drone_ids();
    unset[0].expected = unset[0].got = 0u;
    const eg::Verdict b = eg::identities_kept("propulsion-kept", unset, eg::kDroneIdentities);
    EXPECT_FALSE(b.ok) << b.detail;
    EXPECT_TRUE(has(b.detail, "absent:")) << b.detail;
}

// ===========================================================================
// import-named
// ===========================================================================

TEST(SandboxEndGoalImport, PassesAMeshNamedAsItsFileNamesIt) {
    const eg::Verdict v = eg::import_named(good_import());
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalImport, NamesAnUnnamedMeshByItsFilesStem) {
    eg::ImportShown m = good_import();
    m.mesh_name.clear();
    m.shown_name = "park_bench";
    const eg::Verdict v = eg::import_named(m);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalImport, FailsAWrongNameOrSourceOrSidecar) {
    eg::ImportShown wrong_name = good_import();
    wrong_name.shown_name = "Mesh";
    eg::ImportShown wrong_source = good_import();
    wrong_source.shown_source = "park_bench.gltf";
    eg::ImportShown no_sidecar = good_import();
    no_sidecar.sidecar_loaded = false;
    eg::ImportShown short_sidecar = good_import();
    short_sidecar.sidecar_triangles = 1999;
    for (const eg::ImportShown* m : {&wrong_name, &wrong_source, &no_sidecar, &short_sidecar}) {
        const eg::Verdict v = eg::import_named(*m);
        EXPECT_TRUE(judged_fail(v)) << v.detail;
    }
}

TEST(SandboxEndGoalImport, IsAbsentWithNoNameToExpectOrNoTriangles) {
    eg::ImportShown nameless = good_import();
    nameless.mesh_name.clear();
    nameless.file.clear();
    nameless.shown_source.clear();
    nameless.shown_name.clear();
    eg::ImportShown empty = good_import();
    empty.gltf_triangles = empty.sidecar_triangles = 0;
    for (const eg::ImportShown* m : {&nameless, &empty}) {
        const eg::Verdict v = eg::import_named(*m);
        EXPECT_FALSE(v.ok) << v.detail;
        EXPECT_TRUE(has(v.detail, "absent:")) << v.detail;
    }
}

// ===========================================================================
// region-resolved, publishes, run-line-shown
// ===========================================================================

TEST(SandboxEndGoalRegion, PassesTheResolveShownNumberForNumber) {
    const eg::GridNumbers g{32, 32, 32, 0.025f, 3.048f};
    const eg::Verdict v = eg::region_resolved(g, g);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalRegion, FailsAShownGridThatIsNotTheResolve) {
    const eg::GridNumbers resolved{32, 32, 32, 0.025f, 3.048f};
    eg::GridNumbers shown = resolved;
    shown.dx = 0.0125f;
    const eg::Verdict v = eg::region_resolved(shown, resolved);
    EXPECT_TRUE(judged_fail(v)) << v.detail;
    const eg::Verdict none = eg::region_resolved(shown, eg::GridNumbers{});
    EXPECT_FALSE(none.ok) << none.detail;
    EXPECT_TRUE(has(none.detail, "absent:")) << none.detail;
}

TEST(SandboxEndGoalRegion, ReadsAPublishedVolume) {
    spade::FlowVolume v;
    v.header.cell = 0.1f;
    v.header.nx = v.header.ny = v.header.nz = 2;
    v.v.assign(8u * spade::kFlowHalvesPerCell, 0u);
    v.s.assign(8u * spade::kFlowHalvesPerCell, 0u);
    v.generation = spade::next_flow_generation();
    const eg::Published p = eg::published(v);
    EXPECT_EQ(p.generation, v.generation);
    EXPECT_TRUE(p.valid);
    EXPECT_EQ(p.no_data, 0u);
    v.header.cell = 0.0f;
    EXPECT_FALSE(eg::published(v).valid);
}

TEST(SandboxEndGoalRegion, PassesANewValidVolumeEveryFrame) {
    const std::vector<eg::Published> frames = {{11, true, 0}, {12, true, 0}, {15, true, 0}};
    const eg::Verdict v = eg::publishes(frames);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalRegion, FailsARepeatedOrOlderOrInvalidOrHoledVolume) {
    const std::vector<std::vector<eg::Published>> bad = {{{11, true, 0}, {11, true, 0}},
                                                         {{11, true, 0}, {12, true, 0}, {11, true, 0}},
                                                         {{11, true, 0}, {12, false, 0}},
                                                         {{11, true, 0}, {12, true, 3}},
                                                         {{0, true, 0}}};
    for (const auto& frames : bad) {
        const eg::Verdict v = eg::publishes(frames);
        EXPECT_TRUE(judged_fail(v)) << v.detail;
    }
    const eg::Verdict none = eg::publishes({});
    EXPECT_FALSE(none.ok) << none.detail;
    EXPECT_TRUE(has(none.detail, "absent:")) << none.detail;
}

TEST(SandboxEndGoalRegion, TheRunLineShownIsLiveRunsExactly) {
    const std::string line = "actuator disc \xc2\xb7 32\xc2\xb3, dx 25 mm, R/dx 3 \xc2\xb7 t = 0.250 s";
    EXPECT_TRUE(eg::run_line_shown(line, line).ok);
    EXPECT_TRUE(judged_fail(eg::run_line_shown("actuator disc", line)));
    const eg::Verdict none = eg::run_line_shown(line, "");
    EXPECT_FALSE(none.ok) << none.detail;
    EXPECT_TRUE(has(none.detail, "absent:")) << none.detail;
}

// ===========================================================================
// editor.views
// ===========================================================================

TEST(SandboxEndGoalViews, PassesEachModeDrawnAndChangedFromTheLast) {
    const Image shaded = drawn({200, 180, 40});
    const Image wire = drawn({240, 240, 240}, 30);
    const Image vortices = drawn({10, 200, 90}, 44);
    eg::ModeShown refused;
    refused.kind = "exact SDF";
    refused.why_not = "the exact SDF is drawn on the CPU only";
    refused.shown_text = refused.why_not;
    const std::vector<eg::ModeShown> modes = {mode("shaded", shaded), mode("wireframe", wire), refused,
                                              mode("vortices", vortices)};
    const eg::Verdict v = eg::modes_drawn(modes);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalViews, FailsAModeUnchangedFromTheLastWhicheverTheStepCallsFirst) {
    // The check, not the step, decides which mode is first: the second mode
    // repeats the first's frame and fails.
    const Image shaded = drawn({200, 180, 40});
    const std::vector<eg::ModeShown> modes = {mode("shaded", shaded), mode("wireframe", shaded)};
    const eg::Verdict v = eg::modes_drawn(modes);
    EXPECT_TRUE(judged_fail(v)) << v.detail;
}

TEST(SandboxEndGoalViews, FailsABlankOrUnhonouredOrUnappliedMode) {
    const Image shaded = drawn({200, 180, 40});
    const Image blank = image(64, 48, kClear);
    const Image other = drawn({10, 200, 90}, 30);
    eg::ModeShown unhonoured = mode("vortices", other);
    unhonoured.unhonoured = "mode";
    eg::ModeShown unapplied = mode("vortices", other);
    unapplied.applied = false;
    const std::vector<std::vector<eg::ModeShown>> cases = {
        {mode("shaded", shaded), mode("wireframe", blank)},
        {mode("shaded", shaded), unhonoured},
        {mode("shaded", shaded), unapplied},
    };
    for (const auto& modes : cases) {
        const eg::Verdict v = eg::modes_drawn(modes);
        EXPECT_TRUE(judged_fail(v)) << v.detail;
    }
}

TEST(SandboxEndGoalViews, ARefusedKindShowsItsWhyNotVerbatim) {
    eg::ModeShown refused;
    refused.kind = "exact SDF";
    refused.why_not = "the exact SDF is drawn on the CPU only";
    refused.shown_text = "";
    const eg::Verdict blank = eg::modes_drawn(std::vector<eg::ModeShown>{refused});
    EXPECT_TRUE(judged_fail(blank)) << blank.detail;
    refused.why_not.clear();
    const eg::Verdict none = eg::modes_drawn(std::vector<eg::ModeShown>{refused});
    EXPECT_FALSE(none.ok) << none.detail;
    EXPECT_TRUE(has(none.detail, "mis-staged:")) << none.detail;
}

TEST(SandboxEndGoalViews, ModesOfTwoSizesAreMisStaged) {
    const Image a = drawn({200, 180, 40});
    const Image b = image(32, 24, kClear, {240, 240, 240}, 2, 2, 20, 20);
    const eg::Verdict v = eg::modes_drawn(std::vector<eg::ModeShown>{mode("shaded", a), mode("wireframe", b)});
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "mis-staged:")) << v.detail;
}

TEST(SandboxEndGoalViews, PassesTheLegendsApplyViewReturned) {
    const eg::Verdict v = eg::legend_shown(legends({speed_legend(12.0f)}));
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalViews, FailsAnotherLegendOrAStillField) {
    eg::LegendCheck other = legends({speed_legend(12.0f)});
    other.shown[0].unit = "km/h";
    eg::LegendCheck header = legends({speed_legend(12.0f)});
    header.shown[0].range.from_header = false;
    eg::LegendCheck still = legends({speed_legend(0.3f)});
    eg::LegendCheck just_under = legends({speed_legend(0.99f)});
    eg::LegendCheck flat = legends({speed_legend(0.0f)});
    eg::LegendCheck missing = legends({speed_legend(12.0f), speed_legend(12.0f)});
    missing.shown.pop_back();
    for (const eg::LegendCheck* c : {&other, &header, &still, &just_under, &flat, &missing}) {
        const eg::Verdict v = eg::legend_shown(*c);
        EXPECT_TRUE(judged_fail(v)) << v.detail;
    }
    EXPECT_TRUE(eg::legend_shown(legends({speed_legend(1.0f)})).ok);
}

TEST(SandboxEndGoalViews, ADroppedLayersLegendIsAbsent) {
    eg::LegendCheck dropped = legends({});
    dropped.layers = 1;  // the spec has a layer; apply_view returned nothing
    const eg::Verdict v = eg::legend_shown(dropped);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "absent:")) << v.detail;
}

TEST(SandboxEndGoalViews, AFixedRangeWhileTheRegionRunsIsMisStaged) {
    // 0 to 12 m/s, fixed: still air would read the same.
    const eg::Verdict v = eg::legend_shown(legends({speed_legend(12.0f, false)}));
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "mis-staged:")) << v.detail;
    EXPECT_TRUE(eg::legend_shown(legends({speed_legend(12.0f, false)}, false)).ok);
}

TEST(SandboxEndGoalViews, PassesTwoDifferentViewsSideBySide) {
    const eg::PixelsView bg = clear_background(64, 48).view();
    const Image left = drawn({200, 180, 40});
    const Image right = drawn({240, 240, 240}, 30);
    const eg::Verdict v = eg::two_views("shaded", "wireframe+vortex_iso", left.view(), right.view(), bg, bg);
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalViews, FailsOneSpecTwiceOrABlankOrAMatchingPair) {
    const eg::PixelsView bg = clear_background(64, 48).view();
    const Image a = drawn({200, 180, 40});
    const Image b = drawn({240, 240, 240}, 30);
    const Image blank = image(64, 48, kClear);
    const eg::Verdict same_spec = eg::two_views("shaded", "shaded", a.view(), b.view(), bg, bg);
    const eg::Verdict one_blank = eg::two_views("shaded", "wireframe", a.view(), blank.view(), bg, bg);
    const eg::Verdict matching = eg::two_views("shaded", "wireframe", a.view(), a.view(), bg, bg);
    for (const eg::Verdict* v : {&same_spec, &one_blank, &matching}) EXPECT_TRUE(judged_fail(*v)) << v->detail;
}

TEST(SandboxEndGoalViews, TwoViewsOfTwoSizesAreMisStaged) {
    const eg::PixelsView bg = clear_background(64, 48).view();
    const Image a = drawn({200, 180, 40});
    const Image b = image(32, 24, kClear, {240, 240, 240}, 2, 2, 20, 20);
    const eg::Verdict v = eg::two_views("shaded", "wireframe", a.view(), b.view(), bg, bg);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "mis-staged:")) << v.detail;
}

// ===========================================================================
// editor.save_reopen
// ===========================================================================

TEST(SandboxEndGoalRoundTrip, PassesAnExactRoundTrip) {
    const eg::Verdict v = eg::round_trip(good_round_trip());
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxEndGoalRoundTrip, FailsAByteOrAnIdentityOrABodyThatDiffers) {
    eg::RoundTrip byte = good_round_trip();
    byte.reemitted[17] = 'X';
    eg::RoundTrip identity = good_round_trip();
    identity.identities[1].got = 0x2223u;
    eg::RoundTrip ulp = good_round_trip();
    ulp.bodies_after[1].vel.y = std::nextafter(ulp.bodies_after[1].vel.y, 1.0f);
    eg::RoundTrip zero = good_round_trip();
    zero.bodies_before[0].vel.x = 0.0f;
    zero.bodies_after[0].vel.x = -0.0f;  // equal by ==, not bit for bit
    eg::RoundTrip mass = good_round_trip();
    mass.bodies_after[0].mass = 0.5f;  // a reader that drops the mass matches at rest
    eg::RoundTrip inertia = good_round_trip();
    inertia.bodies_after[1].inv_inertia_diag.y = 0.0f;
    eg::RoundTrip proxy = good_round_trip();
    proxy.bodies_after[0].proxy_radius = 0.2f;
    eg::RoundTrip flags = good_round_trip();
    flags.bodies_after[0].flags = 1u;
    eg::RoundTrip count = good_round_trip();
    count.bodies_after.pop_back();
    for (const eg::RoundTrip* t : {&byte, &identity, &ulp, &zero, &mass, &inertia, &proxy, &flags, &count}) {
        const eg::Verdict v = eg::round_trip(*t);
        EXPECT_TRUE(judged_fail(v)) << v.detail;
    }
}

TEST(SandboxEndGoalRoundTrip, IsAbsentWhenADronesIdentityIsLeftOut) {
    eg::RoundTrip t = good_round_trip();
    t.identities.pop_back();  // no "drone 0 propulsion"
    const eg::Verdict v = eg::round_trip(t);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "absent:") && has(v.detail, "drone 0 propulsion")) << v.detail;
}

TEST(SandboxEndGoalRoundTrip, IsMisStagedOrAbsentWhenTheRunsDoNotMatch) {
    eg::RoundTrip short_run = good_round_trip();
    short_run.steps_after = 59;
    const eg::Verdict a = eg::round_trip(short_run);
    EXPECT_FALSE(a.ok) << a.detail;
    EXPECT_TRUE(has(a.detail, "mis-staged:")) << a.detail;
    eg::RoundTrip no_bodies = good_round_trip();
    no_bodies.bodies_before.clear();
    no_bodies.bodies_after.clear();
    const eg::Verdict b = eg::round_trip(no_bodies);
    EXPECT_FALSE(b.ok) << b.detail;
    EXPECT_TRUE(has(b.detail, "absent:")) << b.detail;
    eg::RoundTrip no_drone = good_round_trip();
    no_drone.drones = 0;
    const eg::Verdict c = eg::round_trip(no_drone);
    EXPECT_FALSE(c.ok) << c.detail;
    EXPECT_TRUE(has(c.detail, "mis-staged:")) << c.detail;
}

// Interface's re-review of 399390e.

TEST(SandboxEndGoalLibrary, FailsAThumbnailWithOneWrongPixel) {
    const std::vector<Image> files = nine_thumbnails();
    std::vector<Image> shown = files;
    shown[6].set(40, 20, {255, 0, 255});
    const eg::Verdict v = eg::thumbnails(views_of(shown), views_of(files));
    EXPECT_TRUE(judged_fail(v)) << v.detail;
    std::vector<Image> patched = files;
    for (uint32_t x = 0; x < 6; ++x) {
        for (uint32_t y = 0; y < 5; ++y) patched[6].set(30 + x, 20 + y, {255, 0, 255});  // 30 pixels: 0.98%
    }
    const eg::Verdict p = eg::thumbnails(views_of(patched), views_of(files));
    EXPECT_TRUE(judged_fail(p)) << p.detail;
}

TEST(SandboxEndGoalLibrary, ANoisyFlatPlaceholderIsOneColour) {
    // Grey with +-4 levels of noise in each channel (729 values over 3,072
    // pixels, about 4 each), plus a 30-pixel speck: the speck is the most
    // common EXACT value, so judging by it reads the grey as varied. The
    // colour most pixels sit within 8 levels of is a grey, and only the
    // speck, 0.98%, is off it: one colour.
    std::vector<Image> files = nine_thumbnails();
    Image noisy = image(64, 48, {128, 128, 128});
    for (uint32_t y = 0; y < 48; ++y) {
        for (uint32_t x = 0; x < 64; ++x) {
            const uint32_t h = (x * 73856093u) ^ (y * 19349663u) ^ ((x + 64u * y) * 83492791u);
            noisy.set(x, y, {static_cast<uint8_t>(124 + h % 9), static_cast<uint8_t>(124 + (h / 9) % 9),
                             static_cast<uint8_t>(124 + (h / 81) % 9)});
        }
    }
    for (uint32_t x = 0; x < 30; ++x) noisy.set(x, 0, {10, 200, 10});
    files[1] = noisy;
    const eg::Verdict v = eg::thumbnails(views_of(files), views_of(files));
    EXPECT_TRUE(judged_fail(v)) << v.detail;
    EXPECT_TRUE(has(v.detail, "thumbnail 1")) << v.detail;
}

TEST(SandboxEndGoalIdentities, ADuplicateNameIsMisStaged) {
    auto ids = drone_ids();
    ids.push_back({"propulsion", 0xC3u, 0xDEADu});  // the same name again, another value
    const eg::Verdict v = eg::identities_kept("propulsion-kept", ids, eg::kDroneIdentities);
    EXPECT_FALSE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "mis-staged:")) << v.detail;
}

TEST(SandboxEndGoalViews, FewerThanTwoModesDrawnIsAbsent) {
    eg::ModeShown refused;
    refused.kind = "exact SDF";
    refused.why_not = "the exact SDF is drawn on the CPU only";
    refused.shown_text = refused.why_not;
    const Image shaded = drawn({200, 180, 40});
    for (const std::vector<eg::ModeShown>& modes :
         {std::vector<eg::ModeShown>{refused, refused}, std::vector<eg::ModeShown>{mode("shaded", shaded), refused}}) {
        const eg::Verdict v = eg::modes_drawn(modes);
        EXPECT_FALSE(v.ok) << v.detail;
        EXPECT_TRUE(has(v.detail, "absent:")) << v.detail;
    }
}

TEST(SandboxEndGoalViews, NoLayersIsMisStagedAndExtraLegendsAJudgedFail) {
    eg::LegendCheck none = legends({});
    const eg::Verdict a = eg::legend_shown(none);
    EXPECT_FALSE(a.ok) << a.detail;
    EXPECT_TRUE(has(a.detail, "mis-staged:")) << a.detail;
    eg::LegendCheck extra = legends({speed_legend(12.0f), speed_legend(12.0f)});
    extra.layers = 1;
    const eg::Verdict b = eg::legend_shown(extra);
    EXPECT_TRUE(judged_fail(b)) << b.detail;
}

// The views' observables (Interface's editor.views): a mode's background
// frame, and the same mode grabbed again.

TEST(SandboxEndGoalViews, AViewOfItsBackgroundAloneIsBlank) {
    const Image background = sky();
    Image with_items = sky();
    for (uint32_t x = 10; x < 26; ++x) {
        for (uint32_t y = 10; y < 29; ++y) with_items.set(x, y, {200, 180, 40});
    }
    eg::ModeShown bare = mode("wireframe", background);
    bare.background = background.view();
    eg::ModeShown drawn_over = mode("shaded", with_items);
    drawn_over.background = background.view();
    const eg::Verdict v = eg::modes_drawn(std::vector<eg::ModeShown>{drawn_over, bare});
    EXPECT_TRUE(judged_fail(v)) << v.detail;
    EXPECT_TRUE(has(v.detail, "(wireframe) blank")) << v.detail;
}

TEST(SandboxEndGoalViews, AViewThatMovesWithNoModeChangeIsMisStaged) {
    // 100 x 100: 25 pixels moved is 0.25%, the most a still view may change.
    const Image shaded = image(100, 100, kClear, {200, 180, 40}, 10, 10, 40, 40);
    const Image wire = image(100, 100, kClear, {240, 240, 240}, 50, 50, 90, 90);
    for (const uint32_t moved : {25u, 26u}) {
        Image later = shaded;
        for (uint32_t x = 0; x < moved; ++x) later.set(x % 50 + 45, x / 50 + 2, {255, 255, 255});
        eg::ModeShown first = mode("shaded", shaded);
        first.again = later.view();
        const eg::Verdict v = eg::modes_drawn(std::vector<eg::ModeShown>{first, mode("wireframe", wire)});
        if (moved == 25u) {
            EXPECT_TRUE(v.ok) << v.detail;
        } else {
            EXPECT_FALSE(v.ok) << v.detail;
            EXPECT_TRUE(has(v.detail, "mis-staged: the view changes 0.26%")) << v.detail;
        }
    }
}

TEST(SandboxEndGoalViews, AMissingOrMisSizedBackgroundOrSecondGrabIsMisStaged) {
    const Image shaded = drawn({200, 180, 40});
    const Image wire = drawn({240, 240, 240}, 30);
    const Image small = image(32, 24, kClear);
    eg::ModeShown no_background = mode("wireframe", wire);
    no_background.background = {};
    eg::ModeShown small_background = mode("wireframe", wire);
    small_background.background = small.view();
    eg::ModeShown no_again = mode("wireframe", wire);
    no_again.again = {};
    eg::ModeShown small_again = mode("wireframe", wire);
    small_again.again = small.view();
    for (const eg::ModeShown& second : {no_background, small_background, no_again, small_again}) {
        const eg::Verdict v = eg::modes_drawn(std::vector<eg::ModeShown>{mode("shaded", shaded), second});
        EXPECT_FALSE(v.ok) << v.detail;
        EXPECT_TRUE(has(v.detail, "mis-staged:")) << v.detail;
    }
}

TEST(SandboxEndGoalViews, AHalfOfItsBackgroundAloneIsBlankAndNoBackgroundMisStaged) {
    const Image left_bg = sky();
    const Image right = drawn({240, 240, 240}, 30);
    const eg::Verdict bare = eg::two_views("shaded", "wireframe", left_bg.view(), right.view(), left_bg.view(),
                                           clear_background(64, 48).view());
    EXPECT_TRUE(judged_fail(bare)) << bare.detail;
    const eg::Verdict none =
        eg::two_views("shaded", "wireframe", left_bg.view(), right.view(), {}, clear_background(64, 48).view());
    EXPECT_FALSE(none.ok) << none.detail;
    EXPECT_TRUE(has(none.detail, "mis-staged:")) << none.detail;
}

// A pass carries its margins, so one near a threshold shows in the record.
// Each rectangle is 304 of 3,072 pixels, 9.896%; two apart, 19.79%.

TEST(SandboxEndGoalViews, APassNamesItsMargins) {
    const Image shaded = drawn({200, 180, 40});
    const Image wire = drawn({240, 240, 240}, 30);
    const eg::Verdict v = eg::modes_drawn(std::vector<eg::ModeShown>{mode("shaded", shaded), mode("wireframe", wire)});
    ASSERT_TRUE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "changed from the last by at least 19.79%")) << v.detail;
    EXPECT_TRUE(has(v.detail, "off its background by at least 9.896%")) << v.detail;
    EXPECT_TRUE(has(v.detail, "still to 0% between two grabs")) << v.detail;
    const eg::PixelsView bg = clear_background(64, 48).view();
    const eg::Verdict two = eg::two_views("shaded", "wireframe", shaded.view(), wire.view(), bg, bg);
    ASSERT_TRUE(two.ok) << two.detail;
    EXPECT_TRUE(has(two.detail, "off their backgrounds by 9.896% and 9.896%")) << two.detail;
}
