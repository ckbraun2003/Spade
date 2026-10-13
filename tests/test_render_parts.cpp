// test_render_parts.cpp -- drawing a model's parts (spade::render_parts;
// RND-9, docs/design/rendering/plans/2026-10-13-render-parts-plan.md).
// prepare_parts() builds each part's geometry once in its part frame, and
// bind_parts() poses it as Core's posed_parts() poses the part. Expectations
// come from Core's own posing and sampling (posed_parts, sample_parts,
// posed_program) or are worked by hand from the pose, never read back from
// render_parts (TD-4).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "flow_synthetic.hpp"
#include "reference_airframe.hpp"
#include "render/flow_field.hpp"
#include "render/raster_cpu.hpp"
#include "render/raymarch.hpp"
#include "render/scene.hpp"
#include "render_parts/render_parts.hpp"
#include "state/layout.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/part_pose.hpp"
#include "world/mesh.hpp"
#include "world/sdf.hpp"

using spade::render::Camera;
using spade::render::DrawItem;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;
using spade::render_parts::PartsLook;
using spade::render_parts::bind_parts;
using spade::render_parts::prepare_parts;
using spade::vehicles::ModelType;
using spade::vehicles::PartAttach;
using spade::vehicles::PartDesc;
using spade::vehicles::PartFilter;
using spade::vehicles::PartSurface;
using spade::vehicles::PosedParts;

namespace {

constexpr uint32_t kW = 200, kH = 150;

[[nodiscard]] ModelType reference_model() {
    return spade::vehicles::compile_airframe(spade::vehicles::fixtures::reference_quad()).value().model;
}

// The reference quad's rotors, with `parts` in place of its own.
[[nodiscard]] ModelType quad_with(std::vector<PartDesc> parts) {
    ModelType m = reference_model();
    m.parts = std::move(parts);
    return m;
}

[[nodiscard]] spade::BodyState body_at(glm::vec3 pos, glm::quat orient) {
    spade::BodyState b{};
    b.pos = pos;
    b.orient = orient;
    return b;
}

// A tilted pose at a height: 25 degrees about a slanted axis.
[[nodiscard]] spade::BodyState tilted() {
    return body_at(glm::vec3(0.3f, 1.2f, -0.2f),
                   glm::angleAxis(glm::radians(25.0f), glm::normalize(glm::vec3(1.0f, 0.4f, -0.6f))));
}

[[nodiscard]] spade::SdfNode leaf(spade::SdfPrim kind, glm::vec4 params) {
    spade::SdfNode n{};
    n.kind = static_cast<uint32_t>(kind);
    n.op = static_cast<uint32_t>(spade::SdfOp::none);
    n.params = params;
    return n;
}

[[nodiscard]] spade::SdfNode op(spade::SdfOp which) {
    spade::SdfNode n{};
    n.op = static_cast<uint32_t>(which);
    return n;
}

[[nodiscard]] spade::SdfProgram program(std::vector<spade::SdfNode> nodes) {
    spade::SdfProgram p;
    p.transforms.push_back(spade::SdfTransform{});
    p.nodes = std::move(nodes);
    return p;
}

[[nodiscard]] spade::SdfProgram box(glm::vec3 half) {
    return program({leaf(spade::SdfPrim::box, glm::vec4(half, 0.0f))});
}

// A wall part on the body.
[[nodiscard]] PartDesc part(std::string name, spade::SdfProgram shape, glm::vec3 mount_pos = glm::vec3(0.0f),
                            glm::quat mount_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)) {
    return PartDesc{.name = std::move(name),
                    .role = "test",
                    .attach = PartAttach::body,
                    .rotor = 0,
                    .mount_pos = mount_pos,
                    .mount_orient = mount_orient,
                    .shape = std::move(shape),
                    .surface = PartSurface{.kind = PartSurface::Kind::wall, .inv_permeability = 0.0f, .inertial_coeff = 0.0f},
                    .min_feature = 0.001f};
}

[[nodiscard]] PartDesc made_porous(PartDesc p) {
    p.surface = PartSurface{.kind = PartSurface::Kind::porous, .inv_permeability = 1.0e6f, .inertial_coeff = 20.0f};
    return p;
}

[[nodiscard]] PartDesc on_rotor(PartDesc p, uint32_t rotor) {
    p.attach = PartAttach::rotor;
    p.rotor = rotor;
    return p;
}

constexpr uint32_t kBoxTriangles[12][3] = {
    {0, 2, 1}, {1, 2, 3},  // -z
    {4, 5, 6}, {5, 7, 6},  // +z
    {0, 1, 4}, {1, 5, 4},  // -y
    {2, 6, 3}, {3, 6, 7},  // +y
    {0, 4, 2}, {2, 4, 6},  // -x
    {1, 3, 5}, {3, 7, 5},  // +x
};

// A box with half-extents `half` about the part origin, as a mesh.
[[nodiscard]] std::shared_ptr<const spade::TriangleMesh> box_mesh(glm::dvec3 half) {
    std::vector<glm::dvec3> positions;
    for (uint32_t v = 0; v < 8; ++v) {
        positions.push_back(glm::dvec3((v & 1u) != 0 ? half.x : -half.x, ((v >> 1) & 1u) != 0 ? half.y : -half.y,
                                       ((v >> 2) & 1u) != 0 ? half.z : -half.z));
    }
    std::vector<uint32_t> indices;
    for (const auto& t : kBoxTriangles) indices.insert(indices.end(), {t[0], t[1], t[2]});
    const spade::MeshInput input{
        .positions = std::move(positions), .indices = std::move(indices), .unit = spade::MeshUnit::metre};
    return spade::make_mesh(input, "box").value();
}

[[nodiscard]] PartDesc mesh_part(std::string name, std::shared_ptr<const spade::TriangleMesh> mesh,
                                 glm::vec3 mount_pos) {
    PartDesc p = part(std::move(name), spade::SdfProgram{}, mount_pos);
    p.mesh = std::move(mesh);
    return p;
}

// A scene with `materials` plain materials and room for the parts.
[[nodiscard]] RenderScene empty_scene(std::size_t materials = 1) {
    RenderScene scene;
    scene.materials.assign(materials, spade::render::Material{});
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    return scene;
}

// Prepares and binds `model` at `posed` into `scene`, asserting both work.
void bind(const ModelType& model, const PosedParts& posed, const PartsLook& look, RenderScene& scene) {
    const auto prepared = prepare_parts(model);
    ASSERT_TRUE(prepared.has_value()) << prepared.error().context;
    const auto bound = bind_parts(*prepared, posed, look, scene);
    ASSERT_TRUE(bound.has_value()) << bound.error().context;
}

[[nodiscard]] Camera looking_at(glm::vec3 eye, glm::vec3 target) {
    Camera c;
    c.position = eye;
    c.orientation = glm::quatLookAt(glm::normalize(target - eye), glm::vec3(0.0f, 1.0f, 0.0f));
    c.near_plane = 0.01f;
    c.far_plane = 100.0f;
    return c;
}

[[nodiscard]] std::vector<uint8_t> draw(const RenderScene& scene, const Camera& camera) {
    std::vector<uint8_t> px(static_cast<std::size_t>(kW) * kH * 4u, 0u);
    RenderTarget target{.pixels = std::span<uint8_t>(px), .width = kW, .height = kH, .stride = kW * 4u};
    RenderOptions options;
    options.overlays = false;
    options.shadows = false;
    const spade::Result<void> r = spade::render::render(scene, camera, options, target);
    EXPECT_TRUE(r.has_value()) << r.error().context;
    return px;
}

[[nodiscard]] bool differs(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, uint32_t x, uint32_t y) {
    const std::size_t i = (static_cast<std::size_t>(y) * kW + x) * 4u;
    return a[i] != b[i] || a[i + 1] != b[i + 1] || a[i + 2] != b[i + 2];
}

// The draw items bind_parts() appended for each prepared part, in order.
[[nodiscard]] std::vector<std::vector<DrawItem>> items_by_part(const spade::render_parts::PreparedParts& prepared,
                                                               const PosedParts& posed, const RenderScene& scene) {
    std::vector<std::vector<DrawItem>> out;
    std::size_t next = 0;
    for (const spade::vehicles::PosedPart& p : posed.parts) {
        std::vector<DrawItem> items;
        for (std::size_t k = 0; k < prepared.parts[p.part].pieces.size(); ++k) items.push_back(scene.statics[next++]);
        out.push_back(std::move(items));
    }
    EXPECT_EQ(next, scene.statics.size());
    return out;
}

}  // namespace

// Every vertex of every bound mesh, taken to the world by its draw item, lies
// on its own part's surface as Core samples it, the part posed alone (the
// quad's parts overlap: an arm's end sits inside the hull, where the union of
// all parts is below zero). A tessellated primitive's vertices sit on its
// surface, so the band is float rounding at a metre.
TEST(RenderParts, TheReferenceQuadsPartsLieOnThePosedField) {
    const ModelType model = reference_model();
    ASSERT_FALSE(model.parts.empty());
    const auto posed = spade::vehicles::posed_parts(model, tilted(), {}, {}, PartFilter::all);
    ASSERT_TRUE(posed.has_value()) << posed.error().context;
    const auto prepared = prepare_parts(model);
    ASSERT_TRUE(prepared.has_value()) << prepared.error().context;
    RenderScene scene = empty_scene();
    ASSERT_TRUE(bind_parts(*prepared, *posed, PartsLook{}, scene).has_value());
    const auto items = items_by_part(*prepared, *posed, scene);
    ASSERT_EQ(items.size(), posed->parts.size());
    std::size_t vertices = 0;
    float worst = 0.0f;
    for (std::size_t i = 0; i < items.size(); ++i) {
        PosedParts alone = *posed;
        alone.parts = {posed->parts[i]};
        for (const DrawItem& item : items[i]) {
            for (const glm::vec3& v : scene.meshes[item.mesh_index].positions) {
                const glm::vec3 x = glm::vec3(item.local_to_world * glm::vec4(v, 1.0f));
                worst = std::max(worst, std::abs(spade::vehicles::sample_parts(model, alone, x).distance));
                ++vertices;
            }
        }
    }
    EXPECT_GT(vertices, 100u);
    EXPECT_LE(worst, 2.0e-6f) << "the worst vertex is " << worst << " m off its part's surface";
}

// A CPU frame of the bound parts covers the pixels a CPU ray-march of Core's
// posed_program() covers, within a pinned silhouette band (RS4): the raster
// draws tessellated facets, the march the exact field.
TEST(RenderParts, ACpuFrameOfTheReferenceQuadMatchesTheMarchedField) {
    const ModelType model = reference_model();
    const auto posed = spade::vehicles::posed_parts(model, tilted(), {}, {}, PartFilter::all);
    ASSERT_TRUE(posed.has_value()) << posed.error().context;
    const auto field = spade::vehicles::posed_program(model, *posed);
    ASSERT_TRUE(field.has_value()) << field.error().context;
    const glm::vec3 target = posed->com;
    const Camera camera = looking_at(target + glm::vec3(0.15f, 0.12f, 0.20f), target);

    const RenderScene empty = empty_scene();
    RenderScene parts = empty_scene();
    bind(model, *posed, PartsLook{}, parts);
    const std::vector<uint8_t> background = draw(empty, camera);
    const std::vector<uint8_t> drawn = draw(parts, camera);

    const spade::render::RayGrid grid = spade::render::ray_grid(camera, kW, kH);
    std::size_t marched = 0, mismatched = 0, march_only = 0;
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            const glm::vec3 dir = spade::render::camera_ray(grid, x, y).dir_world;
            const bool hit = spade::render::sphere_trace(*field, camera.position, dir, 0.0f, 10.0f).hit;
            const bool drawn_here = differs(background, drawn, x, y);
            marched += hit ? 1u : 0u;
            mismatched += hit != drawn_here ? 1u : 0u;
            march_only += hit && !drawn_here ? 1u : 0u;
        }
    }
    std::printf("[ parts ] reference quad: %zu marched pixels, %zu mismatched (%zu marched only)\n", marched,
                mismatched, march_only);
    // Measured 32 mismatched of 4498 marched (MSVC 1943 release, 2026-10-13),
    // all 32 pixels the march covers and the raster misses: the curved parts
    // (the tube arms and the motors' cylinders) tessellate with their
    // vertices on the surface, so their facets fall just inside the exact
    // silhouette. The CPU raster is the fp64 reference, so another build
    // differs by rounding at most; the band allows 25% over.
    constexpr std::size_t kSilhouetteBand = 40;
    EXPECT_GT(marched, 2000u) << "the quad should fill a good part of the frame";
    EXPECT_LE(mismatched, kSilhouetteBand) << "of " << marched << " marched pixels";
}

// flow_wall marks the walls the solver sees: a wall part on the body, never a
// porous part, and a rotor part only when the look says rotor parts are walls
// (resolved blades, P5). Each part takes its look's material.
TEST(RenderParts, FlowWallMarksWallsOnTheBodyOnly) {
    const ModelType model = quad_with({
        part("hull", box(glm::vec3(0.05f, 0.02f, 0.05f))),
        made_porous(part("guard", box(glm::vec3(0.02f)), glm::vec3(0.0f, -0.04f, 0.0f))),
        on_rotor(part("blade", box(glm::vec3(0.06f, 0.002f, 0.008f))), 0),
        mesh_part("camera", box_mesh(glm::dvec3(0.01, 0.01, 0.02)), glm::vec3(0.06f, 0.0f, 0.0f)),
    });
    const std::vector<float> omega(model.rotors.size(), 800.0f), psi(model.rotors.size(), 0.3f);
    const auto posed = spade::vehicles::posed_parts(model, tilted(), omega, psi, PartFilter::all);
    ASSERT_TRUE(posed.has_value()) << posed.error().context;
    const auto prepared = prepare_parts(model);
    ASSERT_TRUE(prepared.has_value()) << prepared.error().context;
    const PartsLook look{.body = 1, .rotor = 2, .porous = 3, .rotor_parts_are_walls = false};

    RenderScene scene = empty_scene(4);
    ASSERT_TRUE(bind_parts(*prepared, *posed, look, scene).has_value());
    const auto items = items_by_part(*prepared, *posed, scene);
    ASSERT_EQ(items.size(), 4u);
    const bool walls[4] = {true, false, false, true};
    const uint32_t materials[4] = {1, 3, 2, 1};
    for (std::size_t i = 0; i < 4; ++i) {
        ASSERT_FALSE(items[i].empty()) << model.parts[i].name;
        for (const DrawItem& item : items[i]) {
            EXPECT_EQ(item.flow_wall, walls[i]) << model.parts[i].name;
            EXPECT_EQ(item.material_override, materials[i]) << model.parts[i].name;
        }
    }

    PartsLook blades = look;
    blades.rotor_parts_are_walls = true;
    RenderScene resolved = empty_scene(4);
    ASSERT_TRUE(bind_parts(*prepared, *posed, blades, resolved).has_value());
    const auto again = items_by_part(*prepared, *posed, resolved);
    ASSERT_EQ(again.size(), 4u);
    for (const DrawItem& item : again[1]) EXPECT_FALSE(item.flow_wall) << "porous is never a wall";
    for (const DrawItem& item : again[2]) EXPECT_TRUE(item.flow_wall) << "a resolved blade is";
}

// On a CPU frame with a walls_only C_p colouring (RND-8), the wall part takes
// the colour map and the porous part beside it draws as it did without one.
TEST(RenderParts, OnlyTheWallPartsTakeTheCpColour) {
    const ModelType model = quad_with({
        part("wall", box(glm::vec3(0.08f, 0.08f, 0.02f)), glm::vec3(-0.15f, 0.0f, 0.0f)),
        made_porous(part("screen", box(glm::vec3(0.08f, 0.08f, 0.02f)), glm::vec3(0.15f, 0.0f, 0.0f))),
    });
    const auto posed =
        spade::vehicles::posed_parts(model, body_at(glm::vec3(0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)), {}, {},
                                     PartFilter::all);
    ASSERT_TRUE(posed.has_value()) << posed.error().context;
    RenderScene scene = empty_scene();
    bind(model, *posed, PartsLook{}, scene);
    const Camera camera = looking_at(glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f));
    const std::vector<uint8_t> background = draw(empty_scene(), camera);
    const std::vector<uint8_t> plain = draw(scene, camera);

    const spade::FlowVolume volume =
        spade::test::make_flow_volume(glm::vec3(-0.5f), 0.05f, 20, 20, 20, [](glm::vec3 p) {
            spade::test::FlowChannels c{};
            c[spade::kFlowPressure] = 4.0f + 10.0f * p.y;
            return c;
        });
    spade::render::TransferFunction colours;
    colours.quantity = spade::render::FlowQuantity::pressure;
    colours.colours = spade::render::ColourMapId::cool_warm;
    colours.range_min = -6.0f;
    colours.range_max = 6.0f;
    scene.flow_surface_colouring = spade::render::FlowSurfaceColouring{&volume, colours};
    scene.flow_surface_colouring->walls_only = true;
    const std::vector<uint8_t> coloured = draw(scene, camera);

    std::size_t wall = 0, coloured_wall = 0, screen = 0, coloured_screen = 0;
    for (uint32_t y = 0; y < kH; ++y) {
        for (uint32_t x = 0; x < kW; ++x) {
            const bool covered = differs(background, plain, x, y);
            const bool changed = differs(plain, coloured, x, y);
            if (x < kW / 2) {
                wall += covered ? 1u : 0u;
                coloured_wall += changed ? 1u : 0u;
            } else {
                screen += covered ? 1u : 0u;
                coloured_screen += changed ? 1u : 0u;
            }
        }
    }
    ASSERT_GT(wall, 300u) << "the wall is in view";
    ASSERT_GT(screen, 300u) << "the screen is in view";
    EXPECT_GE(coloured_wall * 10u, wall * 9u) << coloured_wall << " of the wall's " << wall << " pixels coloured";
    EXPECT_EQ(coloured_screen, 0u) << "the porous screen draws as before";
}

// A mesh part binds its own triangles: each vertex where the body and then
// the mount put it, worked by hand, and each face's normal pointing out.
TEST(RenderParts, AMeshPartBindsItsOwnTrianglesAtItsPose) {
    const glm::vec3 mount(0.08f, -0.01f, 0.02f);
    const auto mesh = box_mesh(glm::dvec3(0.03, 0.01, 0.02));
    const ModelType model = quad_with({mesh_part("pod", mesh, mount)});
    const spade::BodyState body = tilted();
    const auto posed = spade::vehicles::posed_parts(model, body, {}, {}, PartFilter::all);
    ASSERT_TRUE(posed.has_value()) << posed.error().context;
    RenderScene scene = empty_scene();
    bind(model, *posed, PartsLook{}, scene);
    ASSERT_EQ(scene.statics.size(), 1u);
    ASSERT_TRUE(scene.csg_subtrees.empty());
    const DrawItem& item = scene.statics[0];
    const spade::render::MeshData& drawn = scene.meshes[item.mesh_index];
    const std::span<const uint32_t> indices = mesh->indices();
    ASSERT_EQ(drawn.indices.size(), indices.size());
    const glm::vec3 centre = body.pos + body.orient * mount;
    for (std::size_t j = 0; j < drawn.indices.size(); ++j) {
        const glm::vec3 got = glm::vec3(item.local_to_world * glm::vec4(drawn.positions[drawn.indices[j]], 1.0f));
        const glm::vec3 want = body.pos + body.orient * (mount + mesh->positions()[indices[j]]);
        EXPECT_LE(glm::length(got - want), 1.0e-6f) << "vertex " << j;
        const glm::vec3 n = glm::normalize(glm::mat3(item.local_to_world) * drawn.normals[drawn.indices[j]]);
        EXPECT_GT(glm::dot(n, got - centre), 0.0f) << "vertex " << j << "'s normal points in";
    }
}

// A shape part with a CSG root binds a CsgSubtree: its world program samples
// as Core samples the posed part, and its bounds hold the posed surface.
TEST(RenderParts, ACsgPartMarchesWhereThePartIs) {
    spade::SdfProgram ring = program({leaf(spade::SdfPrim::box, glm::vec4(0.06f, 0.03f, 0.06f, 0.0f)),
                                      leaf(spade::SdfPrim::sphere, glm::vec4(0.045f, 0.0f, 0.0f, 0.0f)),
                                      op(spade::SdfOp::subtract)});
    const ModelType model = quad_with({part("ring", std::move(ring), glm::vec3(0.02f, 0.01f, -0.03f),
                                            glm::angleAxis(glm::radians(30.0f), glm::vec3(0.0f, 0.0f, 1.0f)))});
    const auto posed = spade::vehicles::posed_parts(model, tilted(), {}, {}, PartFilter::all);
    ASSERT_TRUE(posed.has_value()) << posed.error().context;
    RenderScene scene = empty_scene();
    bind(model, *posed, PartsLook{}, scene);
    ASSERT_EQ(scene.csg_subtrees.size(), 1u);
    const spade::render::CsgSubtree& csg = scene.csg_subtrees[0];
    ASSERT_LT(csg.draw_item, scene.statics.size());
    EXPECT_TRUE(scene.statics[csg.draw_item].flow_wall);

    const glm::vec3 lo = csg.bounds.min, hi = csg.bounds.max;
    float worst = 0.0f;
    for (int i = 0; i <= 8; ++i) {
        for (int j = 0; j <= 8; ++j) {
            for (int k = 0; k <= 8; ++k) {
                const glm::vec3 x = lo + (hi - lo) * glm::vec3(i, j, k) / 8.0f;
                const float got = spade::sample(csg.program, x).distance;
                const float want = spade::vehicles::sample_parts(model, *posed, x).distance;
                worst = std::max(worst, std::abs(got - want));
            }
        }
    }
    EXPECT_LE(worst, 1.0e-5f) << "the subtree's world program against the posed part";

    // The drawn mesh sits on the marched surface: each vertex, through its draw
    // item, within the surface-nets band of the program's zero set. This ties
    // the draw item's pose to the program's; the program alone matching Core
    // would pass with a mis-posed mesh.
    const DrawItem& item = scene.statics[csg.draw_item];
    float off_surface = 0.0f;
    for (const glm::vec3& v : scene.meshes[item.mesh_index].positions) {
        const glm::vec3 x = glm::vec3(item.local_to_world * glm::vec4(v, 1.0f));
        off_surface = std::max(off_surface, std::abs(spade::sample(csg.program, x).distance));
        EXPECT_TRUE(glm::all(glm::greaterThanEqual(x, lo - 1.0e-6f)) && glm::all(glm::lessThanEqual(x, hi + 1.0e-6f)))
            << "a mesh vertex outside the subtree's bounds";
    }
    std::printf("[ parts ] CSG ring: mesh vertices at most %.6f m off the marched surface\n",
                static_cast<double>(off_surface));
    // Measured 1.67 mm (MSVC 1943 release, 2026-10-13): surface-nets vertices
    // sit off the zero set at the ring's sharp edges (RS3 keeps the mesh for
    // wireframe and shadow only). A draw item posed rotate-then-translate put
    // them 0.61 m off.
    EXPECT_LE(off_surface, 2.5e-3f) << "the CSG mesh is not where its program is";
}

// What can't be drawn is refused by name, and a refused bind appends nothing.
TEST(RenderParts, RefusesWhatItCannotDrawByName) {
    PartDesc ghost = part("ghost", spade::SdfProgram{});
    const auto unprepared = prepare_parts(quad_with({ghost}));
    ASSERT_FALSE(unprepared.has_value());
    EXPECT_EQ(unprepared.error().code, spade::Code::invalid_argument);
    EXPECT_NE(unprepared.error().context.find("ghost"), std::string::npos) << unprepared.error().context;

    const ModelType one = quad_with({part("hull", box(glm::vec3(0.05f)))});
    const ModelType two = quad_with({part("hull", box(glm::vec3(0.05f))), part("tail", box(glm::vec3(0.02f)))});
    const auto prepared = prepare_parts(one);
    ASSERT_TRUE(prepared.has_value()) << prepared.error().context;
    const auto posed_two = spade::vehicles::posed_parts(two, tilted(), {}, {}, PartFilter::all);
    ASSERT_TRUE(posed_two.has_value());
    RenderScene scene = empty_scene();
    const auto stranger = bind_parts(*prepared, *posed_two, PartsLook{}, scene);
    ASSERT_FALSE(stranger.has_value());
    EXPECT_EQ(stranger.error().code, spade::Code::invalid_argument);
    EXPECT_NE(stranger.error().context.find("posed part 1"), std::string::npos) << stranger.error().context;
    EXPECT_TRUE(scene.statics.empty());
    EXPECT_TRUE(scene.meshes.empty());

    const auto posed_one = spade::vehicles::posed_parts(one, tilted(), {}, {}, PartFilter::all);
    ASSERT_TRUE(posed_one.has_value());
    const auto off_palette = bind_parts(*prepared, *posed_one, PartsLook{.body = 0, .rotor = 0, .porous = 5}, scene);
    ASSERT_FALSE(off_palette.has_value());
    EXPECT_EQ(off_palette.error().code, spade::Code::invalid_argument);
    EXPECT_NE(off_palette.error().context.find("porous"), std::string::npos) << off_palette.error().context;
    EXPECT_TRUE(scene.statics.empty());
}

// Dependencies point down (RND-9): nothing below render_parts includes it.
TEST(RenderParts, NothingBelowIncludesIt) {
    const std::filesystem::path engine(SPADE_ENGINE_DIR);
    std::vector<std::string> found;
    for (const char* module : {"core", "world", "state", "render", "render_gl", "vehicles", "physics", "sim",
                               "scene", "objects", "analysis"}) {
        const std::filesystem::path dir = engine / module;
        if (!std::filesystem::exists(dir)) continue;
        for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
            const std::string ext = entry.path().extension().string();
            if (ext != ".hpp" && ext != ".cpp") continue;
            std::ifstream in(entry.path());
            std::stringstream text;
            text << in.rdbuf();
            if (text.str().find("render_parts/") != std::string::npos) found.push_back(entry.path().string());
        }
    }
    EXPECT_TRUE(found.empty()) << found.size() << " files include render_parts, first " << found.front();
}
