// DrawMode::segmentation and its identity (N13 part 2; docs/design/rendering/
// plans/2026-10-20-sensor-views-design.md): labels on items, the ground and
// bodies; scene_from_world()'s defaults and the spans Interface asked for;
// colours from names alone; and the legend, with its clashes.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <set>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/segmentation.hpp"
#include "world/builder.hpp"

namespace {

using spade::render::DrawItem;
using spade::render::DrawMode;
using spade::render::kNoLabel;
using spade::render::MeshData;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::Rgb8;
using spade::render::SegmentBy;

constexpr uint32_t kSide = 64;
const Rgb8 kBlack{0, 0, 0};

// A wall facing +Z over x in [x0, x1], y in [-1, 1], at z.
[[nodiscard]] MeshData wall(float x0, float x1, float z) {
    MeshData m;
    m.positions = {glm::vec3(x0, -1.0f, z), glm::vec3(x1, -1.0f, z), glm::vec3(x1, 1.0f, z), glm::vec3(x0, 1.0f, z)};
    m.normals.assign(4, glm::vec3(0.0f, 0.0f, 1.0f));
    m.indices = {0, 1, 2, 0, 2, 3};
    return m;
}

// Three walls side by side 3 m away: left, centre, right.
[[nodiscard]] RenderScene three_walls() {
    RenderScene scene;
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-10.0f), .max = glm::vec3(10.0f)};
    scene.materials = {spade::render::Material{}};
    for (const auto& [x0, x1] : {std::pair{-1.6f, -0.6f}, std::pair{-0.5f, 0.5f}, std::pair{0.6f, 1.6f}}) {
        DrawItem item;
        item.mesh_index = static_cast<uint32_t>(scene.meshes.size());
        scene.meshes.push_back(wall(x0, x1, -3.0f));
        scene.statics.push_back(item);
    }
    return scene;
}

struct Frame {
    std::vector<uint8_t> bgrx = std::vector<uint8_t>(kSide * kSide * 4u, 0u);
    [[nodiscard]] Rgb8 at(uint32_t x, uint32_t y) const {
        const std::size_t i = (static_cast<std::size_t>(y) * kSide + x) * 4u;
        return {bgrx[i + 2], bgrx[i + 1], bgrx[i]};
    }
};

[[nodiscard]] spade::Result<Frame> try_draw(const RenderScene& scene, SegmentBy by = SegmentBy::objects) {
    Frame f;
    RenderOptions options;
    options.mode = DrawMode::segmentation;
    options.segment_by = by;
    options.overlays = false;
    spade::render::RenderTarget target{.pixels = std::span<uint8_t>(f.bgrx), .width = kSide, .height = kSide,
                                       .stride = kSide * 4u};
    if (auto drew = spade::render::render(scene, spade::render::Camera{}, options, target); !drew) {
        return std::unexpected(drew.error());
    }
    return f;
}

[[nodiscard]] Frame draw(const RenderScene& scene, SegmentBy by = SegmentBy::objects) {
    auto f = try_draw(scene, by);
    EXPECT_TRUE(f.has_value()) << f.error().context;
    return f ? *f : Frame{};
}

// Pixel columns inside the left, centre and right walls, on the middle row.
constexpr uint32_t kLeft = 12, kCentre = kSide / 2, kRight = kSide - 13, kRow = kSide / 2;

[[nodiscard]] std::string name_of(const RenderScene& scene, uint32_t label) {
    return label < scene.labels.size() ? scene.labels[label] : std::string("<none>");
}

// One colour per name, from the name alone; never black (no surface) or grey
// (unlabelled); and spread, so names mostly read apart.
TEST(Segmentation, AColourIsAFunctionOfTheNameAlone) {
    EXPECT_EQ(spade::render::segment_colour("gate_3"), spade::render::segment_colour(std::string("gate_") + "3"));
    std::set<Rgb8> seen;
    for (int i = 0; i < 200; ++i) {
        const Rgb8 c = spade::render::segment_colour("object " + std::to_string(i));
        EXPECT_NE(c, kBlack);
        EXPECT_FALSE(c[0] == c[1] && c[1] == c[2]) << "a grey for object " << i;
        seen.insert(c);
    }
    EXPECT_GT(seen.size(), 190u) << "names should rarely share a colour";
}

TEST(Segmentation, InternLabelKeepsOneIndexPerName) {
    RenderScene scene;
    const uint32_t a = spade::render::intern_label(scene, "alpha");
    const uint32_t b = spade::render::intern_label(scene, "beta");
    EXPECT_NE(a, b);
    EXPECT_EQ(spade::render::intern_label(scene, "alpha"), a);
    EXPECT_EQ(scene.labels.size(), 2u);
}

// Each item fills flat in its label's colour; an unlabelled item is grey;
// where no surface is, black.
TEST(Segmentation, ItemsDrawFlatInTheirLabelsColours) {
    RenderScene scene = three_walls();
    scene.statics[0].object_label = spade::render::intern_label(scene, "alpha");
    scene.statics[1].object_label = spade::render::intern_label(scene, "beta");
    const Frame f = draw(scene);
    EXPECT_EQ(f.at(kLeft, kRow), spade::render::segment_colour("alpha"));
    EXPECT_EQ(f.at(kLeft, kRow + 9), spade::render::segment_colour("alpha")) << "flat: no shading across the wall";
    EXPECT_EQ(f.at(kCentre, kRow), spade::render::segment_colour("beta"));
    EXPECT_EQ(f.at(kRight, kRow), spade::render::kUnlabelledColour);
    EXPECT_EQ(f.at(kCentre, 1), kBlack);
}

// The colours follow the names, not the order the labels were made in.
TEST(Segmentation, ColoursFollowNamesNotTheirOrder) {
    RenderScene first = three_walls();
    first.statics[0].object_label = spade::render::intern_label(first, "alpha");
    first.statics[2].object_label = spade::render::intern_label(first, "beta");
    RenderScene second = three_walls();
    static_cast<void>(spade::render::intern_label(second, "unused"));
    second.statics[2].object_label = spade::render::intern_label(second, "beta");
    second.statics[0].object_label = spade::render::intern_label(second, "alpha");
    EXPECT_EQ(draw(first).bgrx, draw(second).bgrx);
}

// By parts, an item takes its part's colour, and one with no part its
// object's; by objects, every item its object's.
TEST(Segmentation, APartFallsBackToItsObject) {
    RenderScene scene = three_walls();
    const uint32_t quad = spade::render::intern_label(scene, "quad");
    for (DrawItem& item : scene.statics) item.object_label = quad;
    scene.statics[0].part_label = spade::render::intern_label(scene, "quad/arm_0");
    const Frame parts = draw(scene, SegmentBy::parts);
    EXPECT_EQ(parts.at(kLeft, kRow), spade::render::segment_colour("quad/arm_0"));
    EXPECT_EQ(parts.at(kCentre, kRow), spade::render::segment_colour("quad"));
    const Frame objects = draw(scene, SegmentBy::objects);
    EXPECT_EQ(objects.at(kLeft, kRow), spade::render::segment_colour("quad"));
}

// The analytic ground takes its label's colour, past any mesh.
TEST(Segmentation, TheGroundTakesItsLabel) {
    RenderScene scene;
    scene.materials = {spade::render::Material{}};
    scene.ground_planes.push_back(spade::render::GroundPlane{.normal = glm::vec3(0.0f, 1.0f, 0.0f), .offset = -1.0f});
    scene.ground_planes[0].label = spade::render::intern_label(scene, "ground");
    const Frame f = draw(scene);
    EXPECT_EQ(f.at(kCentre, kSide - 2), spade::render::segment_colour("ground"));
    EXPECT_EQ(f.at(kCentre, 2), kBlack);
}

// A world with a sphere, a ground plane, a CSG gate and a prop.
[[nodiscard]] spade::WorldDesc labelled_world() {
    spade::WorldBuilder b;
    b.name("labels").capacities(spade::Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1});
    b.material(spade::MaterialDesc{.name = "default"});
    b.sphere(0.3f, spade::SdfPose{.position = glm::vec3(-1.0f, 0.0f, 0.0f)});  // node 0
    b.plane(glm::vec3(0.0f, 1.0f, 0.0f), -1.0f);                              // node 1
    b.union_();                                                               // node 2
    b.box(glm::vec3(0.4f), spade::SdfPose{.position = glm::vec3(1.0f, 0.0f, 0.0f)});     // node 3
    b.sphere(0.25f, spade::SdfPose{.position = glm::vec3(1.3f, 0.0f, 0.0f)});            // node 4
    b.subtract();                                                             // node 5: the CSG root
    b.union_();                                                               // node 6
    b.prop("props/ball", spade::SdfPose{.position = glm::vec3(0.0f, 1.0f, 0.0f)}, 0);
    auto world = b.build();
    EXPECT_TRUE(world.has_value()) << world.error().context;
    return world ? *world : spade::WorldDesc{};
}

[[nodiscard]] std::vector<spade::render::NamedMesh> ball() {
    MeshData tri;
    tri.positions = {glm::vec3(0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)};
    tri.normals.assign(3, glm::vec3(0.0f, 0.0f, 1.0f));
    tri.indices = {0, 1, 2};
    return {spade::render::NamedMesh{"props/ball", tri}};
}

[[nodiscard]] std::multiset<std::string> item_names(const RenderScene& scene) {
    std::multiset<std::string> names;
    for (const DrawItem& item : scene.statics) names.insert(name_of(scene, item.object_label));
    return names;
}

TEST(Segmentation, SceneFromWorldNamesItsItemsByDefault) {
    const spade::WorldDesc world = labelled_world();
    ASSERT_EQ(world.sdf.nodes.size(), 7u);
    const auto meshes = ball();
    auto scene = spade::render::scene_from_world(world, meshes, spade::render::SceneLabels{});
    ASSERT_TRUE(scene.has_value()) << scene.error().context;
    EXPECT_EQ(item_names(*scene),
              (std::multiset<std::string>{"sdf node 0", "ground", "sdf node 5", "prop 0: props/ball"}));
    ASSERT_EQ(scene->ground_planes.size(), 1u);
    EXPECT_EQ(name_of(*scene, scene->ground_planes[0].label), "ground");
}

// Interface's spans: a leaf's item takes its node's name, a CSG root's item
// its root node's, the plane's mesh item and analytic ground both the plane's.
TEST(Segmentation, SceneFromWorldTakesTheCallersNames) {
    const spade::WorldDesc world = labelled_world();
    const std::vector<std::string> nodes = {"rock", "floor", "-", "gate", "gate", "gate", "-"};
    const std::vector<std::string> props = {"ball_1"};
    const auto meshes = ball();
    auto scene = spade::render::scene_from_world(world, meshes, spade::render::SceneLabels{nodes, props});
    ASSERT_TRUE(scene.has_value()) << scene.error().context;
    EXPECT_EQ(item_names(*scene), (std::multiset<std::string>{"rock", "floor", "gate", "ball_1"}));
    EXPECT_EQ(name_of(*scene, scene->ground_planes[0].label), "floor");
    EXPECT_EQ(std::count(scene->labels.begin(), scene->labels.end(), "gate"), 1) << "interned once";
}

TEST(Segmentation, SceneFromWorldRefusesABadSpanByName) {
    const spade::WorldDesc world = labelled_world();
    const auto meshes = ball();
    const std::vector<std::string> short_nodes = {"rock"};
    auto refused = spade::render::scene_from_world(world, meshes, spade::render::SceneLabels{short_nodes, {}});
    ASSERT_FALSE(refused.has_value());
    EXPECT_NE(refused.error().context.find("node"), std::string::npos) << refused.error().context;
    const std::vector<std::string> unnamed = {"rock", "", "-", "gate", "gate", "gate", "-"};
    auto empty = spade::render::scene_from_world(world, meshes, spade::render::SceneLabels{unnamed, {}});
    ASSERT_FALSE(empty.has_value());
    EXPECT_NE(empty.error().context.find("node 1"), std::string::npos) << empty.error().context;
    const std::vector<std::string> two_props = {"a", "b"};
    auto props = spade::render::scene_from_world(world, meshes, spade::render::SceneLabels{{}, two_props});
    ASSERT_FALSE(props.has_value());
    EXPECT_NE(props.error().context.find("prop"), std::string::npos) << props.error().context;
}

TEST(Segmentation, BodiesAreNamedByTheirPoseOrTheirIndex) {
    RenderScene scene;
    std::vector<spade::render::BodyPose> bodies(2);
    bodies[0].object_label = spade::render::intern_label(scene, "drone_a");
    spade::render::update_dynamics(scene, bodies);
    ASSERT_EQ(scene.dynamics.size(), 2u);
    EXPECT_EQ(name_of(scene, scene.dynamics[0].object_label), "drone_a");
    EXPECT_EQ(name_of(scene, scene.dynamics[1].object_label), "body 1");
    spade::render::update_dynamics(scene, bodies);
    EXPECT_EQ(scene.labels.size(), 2u) << "a frame's update interns no name twice";
}

// The legend: each name drawn, by name, with its colour; how many items draw
// grey; and every pair of names whose colours clash.
TEST(Segmentation, TheLegendListsNamesColoursUnlabelledAndClashes) {
    // Two names whose colours clash, found by search: they exist (a hash), and
    // the legend must say so.
    std::string a, b;
    std::vector<std::pair<std::string, Rgb8>> tried;
    for (int i = 0; i < 4000 && a.empty(); ++i) {
        const std::string name = "c" + std::to_string(i);
        const Rgb8 c = spade::render::segment_colour(name);
        for (const auto& [other, oc] : tried) {
            int worst = 0;
            for (int k = 0; k < 3; ++k) worst = std::max(worst, std::abs(int(c[k]) - int(oc[k])));
            if (worst <= spade::render::kSegmentClashLevels) {
                a = std::min(name, other);
                b = std::max(name, other);
                break;
            }
        }
        tried.emplace_back(name, c);
    }
    ASSERT_FALSE(a.empty()) << "no clashing pair in 4000 names";

    RenderScene scene = three_walls();
    scene.statics[0].object_label = spade::render::intern_label(scene, b);
    scene.statics[1].object_label = spade::render::intern_label(scene, a);
    scene.statics.push_back(scene.statics[0]);  // b again: one entry
    scene.ground_planes.push_back(spade::render::GroundPlane{.normal = glm::vec3(0.0f, 1.0f, 0.0f), .offset = -1.0f});
    scene.ground_planes[0].label = spade::render::intern_label(scene, "ground");
    static_cast<void>(spade::render::intern_label(scene, "drawn by nothing"));
    const spade::render::SegmentLegend legend = spade::render::segment_legend(scene, SegmentBy::objects);
    std::vector<std::string> names;
    for (const auto& e : legend.entries) {
        names.push_back(e.name);
        EXPECT_EQ(e.colour, spade::render::segment_colour(e.name)) << e.name;
    }
    std::vector<std::string> want = {a, b, "ground"};
    std::sort(want.begin(), want.end());
    EXPECT_EQ(names, want);
    EXPECT_EQ(legend.unlabelled_items, 1u);
    ASSERT_EQ(legend.clashes.size(), 1u);
    EXPECT_EQ(legend.clashes[0].a, a);
    EXPECT_EQ(legend.clashes[0].b, b);
}

// A label past the scene's names is refused by name, never drawn grey.
TEST(Segmentation, ALabelPastTheNamesIsRefused) {
    RenderScene scene = three_walls();
    static_cast<void>(spade::render::intern_label(scene, "alpha"));
    scene.statics[1].object_label = 7;
    auto refused = try_draw(scene);
    ASSERT_FALSE(refused.has_value());
    EXPECT_NE(refused.error().context.find("label 7"), std::string::npos) << refused.error().context;
}

}  // namespace
