// render/flow_field.hpp: slices of a flow volume as field layers, and
// surface colouring by a flow quantity. Checked against linear fields that
// f16 holds exactly and trilinear sampling reproduces exactly.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "cp_sphere.hpp"
#include "flow_synthetic.hpp"
#include "render/flow_field.hpp"
#include "render/raster_cpu.hpp"
#include "render/raymarch.hpp"
#include "render/scene.hpp"
#include "world/builder.hpp"

namespace {


using spade::render::Camera;
using spade::render::ColourMapId;
using spade::render::FieldLayer;
using spade::render::FlowQuantity;
using spade::render::FlowSurfaceColouring;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;
using spade::render::TransferFunction;

// Pressure p = 2x - y + 0.5z, dyadic: f16 holds every centre exactly.
[[nodiscard]] float linear_p(glm::vec3 p) { return 2.0f * p.x - p.y + 0.5f * p.z; }

[[nodiscard]] spade::FlowVolume pressure_volume(glm::vec3 origin, uint32_t nx, uint32_t ny, uint32_t nz,
                                               bool hole = false) {
    return spade::test::make_flow_volume(origin, 0.25f, nx, ny, nz, [hole](glm::vec3 p) {
        spade::test::FlowChannels c{};
        c[spade::kFlowPressure] = hole && p.x > 0.0f ? std::numeric_limits<float>::quiet_NaN() : linear_p(p);
        return c;
    });
}

[[nodiscard]] FieldLayer square(float side, uint32_t cells) {
    FieldLayer layer;
    layer.width = side;
    layer.height = side;
    layer.cells_u = cells;
    layer.cells_v = cells;
    return layer;
}

TEST(RenderFlowField, ASliceSamplesTheFieldAtEachCellCentre) {
    const spade::FlowVolume volume = pressure_volume(glm::vec3(-2.0f), 16, 16, 16);
    FieldLayer geometry = square(1.5f, 6);
    geometry.center = glm::vec3(0.1f, -0.2f, 0.3f);
    geometry.right = glm::normalize(glm::vec3(1.0f, 0.0f, 1.0f));
    const auto slice = spade::render::slice_flow_volume(volume, FlowQuantity::pressure, geometry);
    ASSERT_TRUE(slice.has_value()) << slice.error().context;
    ASSERT_EQ(slice->values.size(), 36u);
    EXPECT_EQ(slice->center, geometry.center);
    for (uint32_t j = 0; j < 6; ++j) {
        for (uint32_t i = 0; i < 6; ++i) {
            const glm::vec3 c = spade::render::field_cell_center(*slice, i, j);
            EXPECT_NEAR(slice->values[j * 6 + i], linear_p(c), 1e-5f) << i << ", " << j;
        }
    }
}

// L6: no-data stays no-data in the slice, and the layer counts it.
TEST(RenderFlowField, NoDataInTheVolumeIsNaNInTheSlice) {
    const spade::FlowVolume volume = pressure_volume(glm::vec3(-2.0f), 16, 16, 16, /*hole=*/true);
    const auto slice = spade::render::slice_flow_volume(volume, FlowQuantity::pressure, square(2.0f, 8));
    ASSERT_TRUE(slice.has_value()) << slice.error().context;
    EXPECT_TRUE(std::isnan(slice->values[7])) << "the right-hand column lies in the hole";
    EXPECT_FALSE(std::isnan(slice->values[0])) << "the left-hand column does not";
    EXPECT_GT(spade::render::count_non_finite(*slice), 0u);
}

TEST(RenderFlowField, APaletteCarriesTheMapsEntries) {
    for (const ColourMapId id : {ColourMapId::viridis, ColourMapId::inferno, ColourMapId::cool_warm}) {
        const auto palette = spade::render::field_palette_of(id);
        const auto& table = spade::render::colour_map_table(id);
        ASSERT_EQ(palette.stops.size(), table.size());
        for (size_t i = 0; i < table.size(); ++i) EXPECT_EQ(palette.stops[i], table[i] * 255.0f) << i;
    }
}

TEST(RenderFlowField, BadSlicesAndColouringsAreRefused) {
    const spade::FlowVolume volume = pressure_volume(glm::vec3(-2.0f), 4, 4, 4);
    EXPECT_FALSE(spade::render::slice_flow_volume(volume, FlowQuantity::pressure, square(1.0f, 0)).has_value())
        << "a slice with no cells";
    spade::FlowVolume bad = volume;
    bad.header.cell = 0.0f;
    EXPECT_FALSE(spade::render::slice_flow_volume(bad, FlowQuantity::pressure, square(1.0f, 2)).has_value())
        << "a malformed volume";
    EXPECT_FALSE(spade::render::validate_flow_surface_colouring(FlowSurfaceColouring{}).has_value()) << "no volume";
    TransferFunction broken;
    broken.range_max = broken.range_min;
    EXPECT_FALSE(spade::render::validate_flow_surface_colouring(FlowSurfaceColouring{&volume, broken}).has_value())
        << "broken colours";
}

// --- Surface colouring through render() -----------------------------------

constexpr uint32_t kSide = 33;

[[nodiscard]] spade::render::MeshData quad(float half) {
    spade::render::MeshData mesh;
    mesh.positions = {glm::vec3(-half, -half, 0.0f), glm::vec3(half, -half, 0.0f), glm::vec3(half, half, 0.0f),
                      glm::vec3(-half, half, 0.0f)};
    mesh.normals.assign(4, glm::vec3(0.0f, 0.0f, 1.0f));
    mesh.indices = {0, 1, 2, 0, 2, 3};
    return mesh;
}

[[nodiscard]] RenderScene quad_scene() {
    RenderScene scene;
    scene.meshes.push_back(quad(1.5f));
    scene.materials = {spade::render::Material{}};
    scene.statics.push_back(spade::render::DrawItem{.mesh_index = 0, .local_to_world = glm::mat4(1.0f),
                                                    .material_override = spade::render::kNoMaterial});
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    return scene;
}

[[nodiscard]] std::vector<uint8_t> draw(const RenderScene& scene, spade::Result<void>* result = nullptr) {
    std::vector<uint8_t> px(static_cast<size_t>(kSide) * kSide * 4u, 0xABu);
    RenderTarget target{.pixels = std::span<uint8_t>(px), .width = kSide, .height = kSide, .stride = kSide * 4u};
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    RenderOptions options;
    options.overlays = false;
    const spade::Result<void> r = spade::render::render(scene, camera, options, target);
    if (result != nullptr) {
        *result = r;
    } else {
        EXPECT_TRUE(r.has_value()) << r.error().context;
    }
    return px;
}

[[nodiscard]] glm::vec3 rgb(const std::vector<uint8_t>& px, uint32_t x, uint32_t y) {
    const size_t i = (static_cast<size_t>(y) * kSide + x) * 4u;
    return glm::vec3(px[i + 2], px[i + 1], px[i + 0]);
}

// A quad at z = 0 in a box x in [-1, 1]: inside the box each pixel takes the
// pressure at its point on the quad through cool-warm over [-2, 2]; outside
// the box, and off the quad, the frame is unchanged.
TEST(RenderFlowField, SurfaceColouringPaintsEachPixelWithItsPointsValue) {
    const spade::FlowVolume volume = pressure_volume(glm::vec3(-1.0f, -2.0f, -1.0f), 8, 16, 8);
    TransferFunction colours;
    colours.quantity = FlowQuantity::pressure;
    colours.colours = ColourMapId::cool_warm;
    colours.range_min = -2.0f;
    colours.range_max = 2.0f;
    RenderScene scene = quad_scene();
    const std::vector<uint8_t> plain = draw(scene);
    scene.flow_surface_colouring = FlowSurfaceColouring{&volume, colours};
    const std::vector<uint8_t> got = draw(scene);

    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    const auto grid = spade::render::ray_grid(camera, kSide, kSide);
    uint32_t checked = 0;
    for (uint32_t x = 0; x < kSide; ++x) {
        const glm::vec3 d = spade::render::camera_ray(grid, x, 16).dir_world;
        const glm::vec3 on_quad = camera.position + d * (5.0f / -d.z);
        if (std::fabs(on_quad.x) < 0.85f) {  // inside the outermost centres: the field is exactly linear
            const glm::vec3 want = 255.0f * spade::render::colour_map_at(ColourMapId::cool_warm, (linear_p(on_quad) + 2.0f) / 4.0f);
            const glm::vec3 diff = glm::abs(rgb(got, x, 16) - want);
            EXPECT_LE(std::max({diff.x, diff.y, diff.z}), 1.0f) << "column " << x;
            ++checked;
        } else if (std::fabs(on_quad.x) > 1.05f) {
            EXPECT_EQ(rgb(got, x, 16), rgb(plain, x, 16)) << "column " << x << " is outside the box";
        }
    }
    EXPECT_GE(checked, 8u);
    EXPECT_EQ(rgb(got, 16, 1), rgb(plain, 16, 1)) << "sky has no surface to colour";
}

// The C_p view's colours (RND-8): C_p's ends land on the map's ends and
// C_p = 0 on p_ref, from the host's q_ref and p_ref.
TEST(RenderFlowField, CpColoursMapTheCoefficientOntoPressure) {
    const float q_ref = 50.0f, p_ref = 10.0f;  // Pa
    const auto made = spade::render::cp_colours(q_ref, p_ref, -1.5f, 1.0f);
    ASSERT_TRUE(made.has_value()) << made.error().context;
    const TransferFunction& tf = *made;
    EXPECT_EQ(tf.quantity, FlowQuantity::pressure);
    EXPECT_EQ(tf.colours, ColourMapId::cool_warm);
    EXPECT_FALSE(tf.auto_range) << "a comparison needs the same mapping in every frame";
    EXPECT_EQ(tf.range_min, -65.0f);  // p_ref - 1.5 q_ref
    EXPECT_EQ(tf.range_max, 60.0f);   // p_ref + 1.0 q_ref
    ASSERT_TRUE(spade::render::validate_transfer_function(tf).has_value());
    spade::FlowVolumeHeader h;
    const auto prepared = spade::render::prepare_transfer(tf, h);
    // C_p at the map's position (cp - cp_min) / (cp_max - cp_min), to float's rounding.
    const auto expect_at = [&](float cp, float t, const char* what) {
        const glm::vec3 got = spade::render::apply_transfer(prepared, p_ref + cp * q_ref).colour;
        const glm::vec3 want = spade::render::colour_map_at(ColourMapId::cool_warm, t);
        EXPECT_NEAR(got.r, want.r, 1e-5f) << what;
        EXPECT_NEAR(got.g, want.g, 1e-5f) << what;
        EXPECT_NEAR(got.b, want.b, 1e-5f) << what;
    };
    expect_at(1.0f, 1.0f, "stagnation, C_p = 1, is the map's top");
    expect_at(-1.5f, 0.0f, "C_p = -1.5 is its bottom");
    expect_at(0.0f, 0.6f, "the ambient, C_p = 0, is 1.5 / 2.5 of the way up");
    const auto viridis = spade::render::cp_colours(q_ref, p_ref, -1.5f, 1.0f, ColourMapId::viridis);
    ASSERT_TRUE(viridis.has_value());
    EXPECT_EQ(viridis->colours, ColourMapId::viridis);
}

// A reference render cannot use is refused by name, never drawn.
TEST(RenderFlowField, CpColoursRefuseABadReference) {
    const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
    struct Case {
        float q_ref, p_ref, cp_min, cp_max;
        const char* why;
    };
    const Case cases[] = {
        {0.0f, 0.0f, -1.0f, 1.0f, "q_ref"},     {-5.0f, 0.0f, -1.0f, 1.0f, "q_ref"},
        {nan, 0.0f, -1.0f, 1.0f, "q_ref"},      {inf, 0.0f, -1.0f, 1.0f, "q_ref"},
        {50.0f, nan, -1.0f, 1.0f, "p_ref"},     {50.0f, 0.0f, 1.0f, 1.0f, "C_p range"},
        {50.0f, 0.0f, 1.0f, -1.0f, "C_p range"}, {50.0f, 0.0f, nan, 1.0f, "C_p range"},
        {1.0e-3f, 1.0e9f, -1.0f, 1.0f, "collapses"},  // a millipascal span on a gigapascal reference
    };
    for (const Case& c : cases) {
        const auto made = spade::render::cp_colours(c.q_ref, c.p_ref, c.cp_min, c.cp_max);
        ASSERT_FALSE(made.has_value()) << c.why;
        EXPECT_EQ(made.error().code, spade::Code::invalid_argument) << c.why;
        EXPECT_NE(made.error().context.find("C_p colours"), std::string::npos) << made.error().context;
        EXPECT_NE(made.error().context.find(c.why), std::string::npos) << made.error().context;
    }
}

// A surface colouring's offset is finite, never negative, and at most
// kMaxSurfaceOffsetCells cells.
TEST(RenderFlowField, AColouringsOffsetIsRefusedOutsideItsRange) {
    const spade::FlowVolume volume = pressure_volume(glm::vec3(-1.0f), 8, 8, 8);
    TransferFunction colours;
    colours.quantity = FlowQuantity::pressure;
    for (const float bad : {std::numeric_limits<float>::quiet_NaN(), -0.5f, 4.5f,
                            std::numeric_limits<float>::infinity()}) {
        FlowSurfaceColouring c{&volume, colours};
        c.offset_cells = bad;
        const auto valid = spade::render::validate_flow_surface_colouring(c);
        ASSERT_FALSE(valid.has_value()) << bad;
        EXPECT_NE(valid.error().context.find("offset"), std::string::npos) << valid.error().context;
    }
    FlowSurfaceColouring fine{&volume, colours};
    fine.offset_cells = spade::render::kMaxSurfaceOffsetCells;
    EXPECT_TRUE(spade::render::validate_flow_surface_colouring(fine).has_value());
    FlowSurfaceColouring at_the_wall{&volume, colours};
    at_the_wall.extrapolate = true;  // with offset 0: two reads at one point
    const auto refused = spade::render::validate_flow_surface_colouring(at_the_wall);
    ASSERT_FALSE(refused.has_value()) << "extrapolating from the wall itself";
    EXPECT_NE(refused.error().context.find("extrapolat"), std::string::npos) << refused.error().context;
}

// Two quads in the box, side by side: the left one a wall in the flow, the
// right one not (an actuator rotor's disc, say). Without walls_only both take
// the colour, as before; with it, only the wall does (RND-8).
TEST(RenderFlowField, OnlyFlowWallsTakeTheColourWhenAsked) {
    const spade::FlowVolume volume = pressure_volume(glm::vec3(-2.0f, -2.0f, -1.0f), 16, 16, 8);
    TransferFunction colours;
    colours.quantity = FlowQuantity::pressure;
    colours.colours = ColourMapId::cool_warm;
    colours.range_min = -6.0f;
    colours.range_max = 6.0f;
    RenderScene scene;
    scene.meshes.push_back(quad(0.8f));
    scene.materials = {spade::render::Material{}};
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    for (const float x : {-1.0f, 1.0f}) {
        spade::render::DrawItem item{.mesh_index = 0,
                                     .local_to_world = glm::translate(glm::mat4(1.0f), glm::vec3(x, 0.0f, 0.0f)),
                                     .material_override = spade::render::kNoMaterial};
        item.flow_wall = x < 0.0f;
        scene.statics.push_back(item);
    }
    const std::vector<uint8_t> plain = draw(scene);
    scene.flow_surface_colouring = FlowSurfaceColouring{&volume, colours};
    const std::vector<uint8_t> every = draw(scene);
    scene.flow_surface_colouring->walls_only = true;
    const std::vector<uint8_t> walls = draw(scene);

    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 5.0f);
    const auto grid = spade::render::ray_grid(camera, kSide, kSide);
    uint32_t wall_px = 0, other_px = 0;
    for (uint32_t x = 0; x < kSide; ++x) {
        const glm::vec3 d = spade::render::camera_ray(grid, x, 16).dir_world;
        const float on_plane = (camera.position + d * (5.0f / -d.z)).x;
        if (on_plane > -1.6f && on_plane < -0.4f) {  // the wall
            EXPECT_NE(rgb(every, x, 16), rgb(plain, x, 16)) << "column " << x;
            EXPECT_EQ(rgb(walls, x, 16), rgb(every, x, 16)) << "column " << x << ": the wall keeps its colour";
            ++wall_px;
        } else if (on_plane > 0.4f && on_plane < 1.6f) {  // not a wall
            EXPECT_NE(rgb(every, x, 16), rgb(plain, x, 16)) << "column " << x;
            EXPECT_EQ(rgb(walls, x, 16), rgb(plain, x, 16)) << "column " << x << ": no wall, no colour";
            ++other_px;
        }
    }
    EXPECT_GE(wall_px, 4u);
    EXPECT_GE(other_px, 4u);
}

// A sheet, Physics' zero-thickness wall: the pressure jumps between the cell
// centres either side of it (z = -0.125 and 0.125 m here), from -above to
// +above, in x and y within [-2, 2] m.
[[nodiscard]] spade::FlowVolume sheet_volume(float above) {
    return spade::test::make_flow_volume(glm::vec3(-2.0f, -2.0f, -1.0f), 0.25f, 16, 16, 8, [above](glm::vec3 p) {
        spade::test::FlowChannels c{};
        c[spade::kFlowPressure] = p.z > 0.0f ? above : -above;
        return c;
    });
}

// Cool-warm over [-1, 1], the sheet's two sides at its ends.
[[nodiscard]] TransferFunction sheet_colours() {
    TransferFunction colours;
    colours.quantity = FlowQuantity::pressure;
    colours.colours = ColourMapId::cool_warm;
    colours.range_min = -1.0f;
    colours.range_max = 1.0f;
    return colours;
}

void expect_map_colour(glm::vec3 got, float t, const char* what) {
    const glm::vec3 want = 255.0f * spade::render::colour_map_at(ColourMapId::cool_warm, t);
    const glm::vec3 diff = glm::abs(got - want);
    EXPECT_LE(std::max({diff.x, diff.y, diff.z}), 1.0f) << what;
}

// Read at the wall, the sample averages the sheet's two sides; read one cell
// off it along the normal (RND-8), it takes the seen side alone.
TEST(RenderFlowField, AnOffsetReadsTheSeenSideOfASheet) {
    const auto centre_colour = [&](float above, float offset) {
        const spade::FlowVolume volume = sheet_volume(above);
        RenderScene scene = quad_scene();  // at z = 0, seen from z = 5
        FlowSurfaceColouring colouring{&volume, sheet_colours()};
        colouring.offset_cells = offset;
        scene.flow_surface_colouring = colouring;
        return rgb(draw(scene), 16, 16);
    };
    expect_map_colour(centre_colour(1.0f, 0.0f), 0.5f, "at the wall: the two sides' mean, 0");
    expect_map_colour(centre_colour(1.0f, 1.0f), 1.0f, "one cell off: the seen side's +1");
    expect_map_colour(centre_colour(-1.0f, 1.0f), 0.0f, "one cell off: the seen side's -1");
}

// The CSG path (RND-8): a 2 cm slab with a hole, ray-marched rather than
// meshed, across the sheet. It takes the colour only as a flow wall, and one
// cell off it, along the distance gradient, it reads the seen side.
TEST(RenderFlowField, ACsgWallTakesTheColourOffItsSeenSide) {
    spade::WorldBuilder b;
    b.name("render-flow-field-csg")
        .capacities(spade::Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1});
    b.box(glm::vec3(1.0f, 1.0f, 0.01f));
    b.sphere(0.2f, spade::SdfPose{.position = glm::vec3(0.6f, 0.6f, 0.0f)});
    b.subtract();
    const spade::Result<spade::WorldDesc> world = b.build();
    ASSERT_TRUE(world.has_value()) << world.error().context;
    spade::Result<RenderScene> made = spade::render::scene_from_world(*world, {});
    ASSERT_TRUE(made.has_value()) << made.error().context;
    RenderScene scene = std::move(*made);
    ASSERT_EQ(scene.csg_subtrees.size(), 1u);
    const glm::vec3 shaded = rgb(draw(scene), 16, 16);

    const spade::FlowVolume volume = sheet_volume(1.0f);
    FlowSurfaceColouring colouring{&volume, sheet_colours()};
    colouring.walls_only = true;
    colouring.offset_cells = 1.0f;
    scene.flow_surface_colouring = colouring;
    EXPECT_EQ(rgb(draw(scene), 16, 16), shaded) << "not a flow wall: no colour";
    scene.statics[scene.csg_subtrees[0].draw_item].flow_wall = true;
    expect_map_colour(rgb(draw(scene), 16, 16), 1.0f, "a flow wall, one cell off: the seen side's +1");
}

// --- R2.4: where to read the wall pressure (RND-8) --------------------------

// R2.4's table, measured 2026-10-13 (MSVC release; worst |dC_p| over the
// pixels at least 0.3 face-on, of about 1930):
//
//   R/dx   offset 0          0.5              1      1.5
//    4     1.05, 708 no-data  0.55, 8 no-data  0.70   0.85
//    8     1.04, 772 no-data  0.33, 52 no-data 0.43   0.57
//   16     1.05, 888 no-data  0.19, 40 no-data 0.25   0.35
//
// At the wall, a stencil reaches the deeper solid cells (no-data), and
// about 40% of the sphere goes uncoloured. Half a cell out, some still do.
// From one cell out none does, but a single read is biased by about
// 4.5 (delta dx / a) sin^2(theta): on a curved wall dp/dn = rho u_t^2 / R.
// One cell is chosen, the nearest that colours every pixel.
constexpr uint32_t kChosenCellsPerRadius = 8;
constexpr float kChosenOffsetCells = 1.0f;
constexpr double kChosenWorstCp = 0.44;  // measured 0.432

// Extrapolated, 2 v(delta) - v(2 delta), the same day:
//
//   R/dx   offset 0.5         1
//    4     0.42, 8 no-data    0.45
//    8     0.27, 52 no-data   0.20
//   16     0.14, 40 no-data   0.079
//
// The single read's first-order bias goes, and the error falls about as
// (dx / a)^2 from 8 to 16 cells per radius; a model of the same grid and
// sampler predicted 0.46, 0.20 and 0.076. Pinned at one cell and 8 per
// radius.
constexpr double kChosenExtrapolatedWorstCp = 0.21;  // measured 0.203

// R2.4's measurement. Each pixel on the sphere (tests/cp_sphere.hpp), its
// colour turned back into C_p through the map, against 1 - 9/4 sin^2(theta) at
// its point, for read offsets of 0 to 1.5 cells at 4, 8 and 16 cells per
// radius. It prints the table above, and pins the chosen offset's worst error
// and full coverage.
TEST(RenderFlowField, ReadingOffTheWallRecoversASpheresSurfaceCp) {
    constexpr uint32_t kPx = 129;
    const float cp_min = -1.25f, cp_max = 1.0f;
    const auto colours = spade::render::cp_colours(0.5f, 0.0f, cp_min, cp_max);
    ASSERT_TRUE(colours.has_value());
    RenderScene scene;
    scene.meshes.push_back(spade::test::potential_sphere_mesh());
    scene.materials = {spade::render::Material{}};
    spade::render::DrawItem sphere{.mesh_index = 0, .local_to_world = glm::mat4(1.0f),
                                   .material_override = spade::render::kNoMaterial};
    sphere.flow_wall = true;
    scene.statics.push_back(sphere);
    scene.bounds = spade::render::Aabb{.min = glm::vec3(-5.0f), .max = glm::vec3(5.0f)};
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 2.2f);
    struct Read {
        float offset;
        bool extrapolate;
    };
    const Read reads[] = {{0.0f, false}, {0.5f, false}, {1.0f, false}, {1.5f, false}, {0.5f, true}, {1.0f, true}};
    spade::test::CpError chosen, chosen_extrapolated;
    for (const uint32_t per_radius : {4u, 8u, 16u}) {
        const spade::FlowVolume volume = spade::test::potential_sphere_volume(per_radius);
        for (const auto& [offset, extrapolate] : reads) {
            FlowSurfaceColouring colouring{&volume, *colours};
            colouring.walls_only = true;
            colouring.offset_cells = offset;
            colouring.extrapolate = extrapolate;
            scene.flow_surface_colouring = colouring;
            std::vector<uint8_t> px(static_cast<size_t>(kPx) * kPx * 4u, 0u);
            RenderTarget target{.pixels = std::span<uint8_t>(px), .width = kPx, .height = kPx, .stride = kPx * 4u};
            RenderOptions options;
            options.overlays = false;
            spade::render::FlowFrameReport report;
            ASSERT_TRUE(spade::render::render(scene, camera, options, target, nullptr, &report).has_value());
            const spade::test::CpError e =
                spade::test::potential_sphere_cp_error(camera, kPx, kPx, cp_min, cp_max, [&](uint32_t x, uint32_t y) {
                    const size_t i = (static_cast<size_t>(y) * kPx + x) * 4u;
                    return glm::vec3(px[i + 2], px[i + 1], px[i + 0]);
                });
            std::printf("[ sphere C_p ] R/dx %2u, offset %.1f%s: worst |dC_p| %.3f at sin^2 %.2f, mean %.4f over %zu "
                        "px; %zu unmapped, %llu no-data\n",
                        per_radius, offset, extrapolate ? " extrapolated" : "", e.worst, e.worst_sin2, e.mean, e.count,
                        e.unmapped, static_cast<unsigned long long>(report.surface_pixels_no_data));
            if (per_radius == kChosenCellsPerRadius && offset == kChosenOffsetCells) {
                (extrapolate ? chosen_extrapolated : chosen) = e;
            }
        }
    }
    EXPECT_LE(chosen.worst, kChosenWorstCp);
    EXPECT_EQ(chosen.unmapped, 0u) << "every pixel on the sphere takes a C_p colour";
    EXPECT_LE(chosen_extrapolated.worst, kChosenExtrapolatedWorstCp);
    EXPECT_EQ(chosen_extrapolated.unmapped, 0u);
}

TEST(RenderFlowField, AMalformedColouringIsRefusedBeforeAnyPixel) {
    RenderScene scene = quad_scene();
    scene.flow_surface_colouring = FlowSurfaceColouring{};
    spade::Result<void> r;
    const std::vector<uint8_t> px = draw(scene, &r);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, spade::Code::invalid_argument);
    EXPECT_TRUE(std::all_of(px.begin(), px.end(), [](uint8_t b) { return b == 0xABu; }));
}

}  // namespace
