// CsgRaster: the CPU raster ray-marches CSG subtrees instead of meshing them.
// Each subtree is marched per pixel within its bounds, not drawn from its
// surface-nets mesh (RS3, replacement signed 2026-10-05;
// rendering/plans/2026-10-04-b2-raymarched-csg-plan.md, step 1). The mesh
// stays for wireframe and shadow casting only.
//
// Each test compares the raster with the reference ray-march
// (render/raymarch, DrawMode::raymarch) on a flat sky, as the agreement
// matrix does (agreement.hpp's precondition).

#include <gtest/gtest.h>

#include <cstdint>
#include <span>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "render/agreement.hpp"
#include "render/raster_cpu.hpp"
#include "render/scene.hpp"
#include "render/target.hpp"
#include "world/builder.hpp"

namespace {

using spade::Capacities;
using spade::MaterialDesc;
using spade::Result;
using spade::SdfPose;
using spade::WorldBuilder;
using spade::WorldDesc;
using spade::render::AgreementResult;
using spade::render::Camera;
using spade::render::DrawMode;
using spade::render::PixelFormat;
using spade::render::RenderOptions;
using spade::render::RenderScene;
using spade::render::RenderTarget;

constexpr uint32_t kWidth = 160, kHeight = 120;

// Rotates a Y-axis cylinder onto the Z axis.
const glm::quat kYToZ(0.70710678f, 0.70710678f, 0.0f, 0.0f);

struct Frame {
    std::vector<uint8_t> storage;
    RenderTarget target;
};

[[nodiscard]] WorldBuilder base_builder() {
    WorldBuilder b;
    b.name("render-csg-raster-test")
        .capacities(Capacities{.bodies = 1, .force_elements = 1, .sensors = 1, .contacts = 1});
    return b;
}

[[nodiscard]] RenderScene scene_or_fail(const WorldDesc& world) {
    Result<RenderScene> scene = spade::render::scene_from_world(world, {});
    if (!scene) {
        ADD_FAILURE() << "scene_from_world failed: " << scene.error().context;
        return RenderScene{};
    }
    scene->lighting.sky_horizon = scene->lighting.sky_zenith;  // a flat sky
    return std::move(*scene);
}

[[nodiscard]] Frame render_frame(const RenderScene& scene, const Camera& camera, DrawMode mode,
                                 bool overlays = false, bool shadows = false) {
    Frame f;
    f.storage.assign(static_cast<size_t>(kWidth) * kHeight * 4u, 0u);
    f.target = RenderTarget{.pixels = std::span<uint8_t>(f.storage), .width = kWidth, .height = kHeight,
                            .stride = kWidth * 4u, .format = PixelFormat::bgrx8};
    RenderOptions options;
    options.mode = mode;
    options.overlays = overlays;
    options.shadows = shadows;
    if (auto drew = spade::render::render(scene, camera, options, f.target); !drew) {
        ADD_FAILURE() << "render() failed: " << drew.error().context;
    }
    return f;
}

[[nodiscard]] AgreementResult agreement(const RenderScene& scene, const Frame& raster, const Frame& reference) {
    return spade::render::compare_silhouettes(raster.target, reference.target,
                                              spade::render::pack_sky_reference_bgrx(scene.lighting));
}

// Sky, red-dominant or blue-dominant: which surface a pixel shows.
enum class Shows { sky, red, blue };

[[nodiscard]] Shows shows(const Frame& f, uint32_t sky, uint32_t x, uint32_t y) {
    const size_t i = (static_cast<size_t>(y) * kWidth + x) * 4u;
    const uint32_t packed = static_cast<uint32_t>(f.storage[i]) | (static_cast<uint32_t>(f.storage[i + 1]) << 8) |
                            (static_cast<uint32_t>(f.storage[i + 2]) << 16) |
                            (static_cast<uint32_t>(f.storage[i + 3]) << 24);
    if (packed == sky) {
        return Shows::sky;
    }
    return f.storage[i + 2] > f.storage[i] ? Shows::red : Shows::blue;  // bgrx: [2] is red, [0] is blue
}

// The share of pixels where the two frames show different surfaces.
[[nodiscard]] double shows_mismatch(const RenderScene& scene, const Frame& a, const Frame& b, uint32_t* blue_in_a) {
    const uint32_t sky = spade::render::pack_sky_reference_bgrx(scene.lighting);
    uint32_t mismatched = 0;
    *blue_in_a = 0;
    for (uint32_t y = 0; y < kHeight; ++y) {
        for (uint32_t x = 0; x < kWidth; ++x) {
            const Shows sa = shows(a, sky, x, y);
            if (sa == Shows::blue) {
                ++*blue_in_a;
            }
            if (sa != shows(b, sky, x, y)) {
                ++mismatched;
            }
        }
    }
    return static_cast<double>(mismatched) / static_cast<double>(kWidth * kHeight);
}

// A red slab with a round hole (a CSG subtree), plus blue spheres drawn as
// tessellated unions. Every node of the slab is red, so the reference's
// per-leaf material and the raster's per-subtree material agree.
[[nodiscard]] WorldDesc slab_and_spheres(glm::vec3 slab_half, const std::vector<SdfPose>& spheres,
                                         const std::vector<float>& radii) {
    WorldBuilder b = base_builder();
    b.material(MaterialDesc{.name = "red", .base_color = {0.9f, 0.1f, 0.1f, 1.0f}});
    b.material(MaterialDesc{.name = "blue", .base_color = {0.1f, 0.1f, 0.9f, 1.0f}});
    b.box(slab_half).material_for_last_node(0);
    b.cylinder(0.4f, 1.0f, SdfPose{.rotation = kYToZ}).material_for_last_node(0);
    b.subtract().material_for_last_node(0);
    for (size_t i = 0; i < spheres.size(); ++i) {
        b.sphere(radii[i], spheres[i]).material_for_last_node(1);
        b.union_().material_for_last_node(1);
    }
    const Result<WorldDesc> world = b.build();
    if (!world) {
        ADD_FAILURE() << "build failed: " << world.error().context;
        return WorldDesc{};
    }
    return *world;
}

// RED before B2. A concentric shell whose wall (0.02 m) is under one B1 cell
// (about 0.05 m on this 8 m subtree): surface nets loses it, and from inside
// the raster shows holes. A sphere tracer never steps past a surface.
TEST(CsgRaster, AWallThinnerThanACellDrawsWithoutHolesFromInside) {
    WorldBuilder b = base_builder();
    b.sphere(4.0f).sphere(3.98f).subtract();
    const Result<WorldDesc> world = b.build();
    ASSERT_TRUE(world) << world.error().context;
    const RenderScene scene = scene_or_fail(*world);

    Camera camera;  // at the centre, looking down -Z: every ray meets the inner wall
    const Frame raster = render_frame(scene, camera, DrawMode::shaded);
    const Frame reference = render_frame(scene, camera, DrawMode::raymarch);
    const AgreementResult a = agreement(scene, raster, reference);

    ASSERT_EQ(a.covered_b, kWidth * kHeight) << "the reference must see the wall in every pixel";
    EXPECT_LT(a.disagreement_fraction, 0.001) << "the raster shows " << (kWidth * kHeight - a.covered_a)
                                              << " holes in a 0.02 m wall";
}

// RED before B2. Shaded and velocity frames draw a CSG subtree without its
// mesh: with the mesh emptied, the subtree still matches the reference.
TEST(CsgRaster, ShadedAndVelocityModesDrawCsgWithoutItsMesh) {
    const WorldDesc world = slab_and_spheres(glm::vec3(1.0f, 1.0f, 0.1f), {}, {});
    RenderScene scene = scene_or_fail(world);
    ASSERT_EQ(scene.statics.size(), 1u) << "the slab is the world's only draw";
    scene.meshes[scene.statics[0].mesh_index].indices.clear();

    Camera camera;
    camera.position = glm::vec3(0.3f, 0.2f, 3.0f);
    const Frame reference = render_frame(scene, camera, DrawMode::raymarch);
    for (const DrawMode mode : {DrawMode::shaded, DrawMode::velocity}) {
        SCOPED_TRACE(mode == DrawMode::shaded ? "shaded" : "velocity");
        const Frame raster = render_frame(scene, camera, mode);
        const AgreementResult a = agreement(scene, raster, reference);
        EXPECT_GT(a.covered_b, kWidth * kHeight / 10u);
        EXPECT_LT(a.disagreement_fraction, 0.001) << "covered " << a.covered_a << " of the reference's "
                                                  << a.covered_b;
    }
}

// Wireframe has no surface to outline in a ray-march, so it keeps drawing
// the mesh (RS3, as signed 2026-10-05).
TEST(CsgRaster, WireframeStillDrawsTheCsgMesh) {
    const WorldDesc world = slab_and_spheres(glm::vec3(1.0f, 1.0f, 0.1f), {}, {});
    RenderScene scene = scene_or_fail(world);
    ASSERT_EQ(scene.statics.size(), 1u);
    Camera camera;
    camera.position = glm::vec3(0.3f, 0.2f, 3.0f);

    const Frame reference = render_frame(scene, camera, DrawMode::raymarch);
    const AgreementResult with_mesh = agreement(scene, render_frame(scene, camera, DrawMode::wireframe), reference);
    EXPECT_GT(with_mesh.covered_a, 0u) << "wireframe draws the mesh's edges";

    scene.meshes[scene.statics[0].mesh_index].indices.clear();
    const AgreementResult without = agreement(scene, render_frame(scene, camera, DrawMode::wireframe), reference);
    EXPECT_EQ(without.covered_a, 0u) << "with the mesh emptied, wireframe draws nothing";
}

// A sphere in front hides part of the slab, and the slab hides part of a
// sphere behind it, except through the hole. The raster must show the same
// surface as the reference almost everywhere: the spheres are tessellated,
// so their silhouettes differ by a pixel here and there. Then each way is
// checked where it decides: inside the front sphere's footprint, where the
// slab lies behind, the raster shows the sphere, never the slab; and the
// back sphere shows through the hole.
TEST(CsgRaster, DepthOrdersCsgAgainstMeshesBothWays) {
    const glm::vec3 slab(1.0f, 1.0f, 0.1f);
    const SdfPose front{.position = {0.6f, 0.5f, 0.6f}};  // wholly over the slab, clear of the hole
    const SdfPose back{.position = {-0.2f, 0.1f, -1.0f}};
    const WorldDesc world = slab_and_spheres(slab, {front, back}, {0.25f, 0.6f});  // outlives scene.sdf
    const RenderScene scene = scene_or_fail(world);
    Camera camera;
    camera.position = glm::vec3(0.0f, 0.0f, 4.0f);

    const Frame raster = render_frame(scene, camera, DrawMode::shaded);
    const Frame reference = render_frame(scene, camera, DrawMode::raymarch);
    uint32_t blue = 0;
    EXPECT_LT(shows_mismatch(scene, raster, reference, &blue), 0.01);

    // The front sphere's footprint, from the reference with that sphere
    // alone, and what lies behind it, from the reference without it.
    WorldBuilder b = base_builder();
    b.material(MaterialDesc{.name = "blue", .base_color = {0.1f, 0.1f, 0.9f, 1.0f}});
    b.sphere(0.25f, front).material_for_last_node(0);
    const Result<WorldDesc> sphere_world = b.build();
    ASSERT_TRUE(sphere_world) << sphere_world.error().context;
    const Frame footprint = render_frame(scene_or_fail(*sphere_world), camera, DrawMode::raymarch);
    const Frame without_front =
        render_frame(scene_or_fail(slab_and_spheres(slab, {back}, {0.6f})), camera, DrawMode::raymarch);

    const uint32_t sky = spade::render::pack_sky_reference_bgrx(scene.lighting);
    uint32_t inside = 0, slab_behind = 0, slab_drawn_over = 0, through_hole = 0;
    for (uint32_t y = 1; y + 1 < kHeight; ++y) {
        for (uint32_t x = 1; x + 1 < kWidth; ++x) {
            bool all_in = true, any_in = false;
            for (uint32_t dy = 0; dy < 3u; ++dy) {
                for (uint32_t dx = 0; dx < 3u; ++dx) {
                    const bool in = shows(footprint, sky, x + dx - 1u, y + dy - 1u) != Shows::sky;
                    all_in = all_in && in;
                    any_in = any_in || in;
                }
            }
            if (all_in) {
                ++inside;
                slab_behind += shows(without_front, sky, x, y) == Shows::red ? 1u : 0u;
                slab_drawn_over += shows(raster, sky, x, y) != Shows::blue ? 1u : 0u;
            } else if (!any_in && shows(raster, sky, x, y) == Shows::blue) {
                ++through_hole;
            }
        }
    }
    ASSERT_GT(inside, 100u) << "the front sphere must cover part of the frame";
    ASSERT_GT(slab_behind, inside * 9u / 10u) << "the slab must lie behind the front sphere";
    EXPECT_EQ(slab_drawn_over, 0u) << slab_drawn_over << " of " << inside
                                   << " pixels inside the front sphere show the slab behind it";
    EXPECT_GT(through_hole, 0u) << "the back sphere shows through the hole";
}

// A camera inside the slab's solid sees what is behind it, as the raster's
// back-face cull (SR-13) and the reference's march-past rule both give.
TEST(CsgRaster, ACameraInsideTheCsgSolidSeesWhatIsBehindIt) {
    const WorldDesc world =
        slab_and_spheres(glm::vec3(2.0f, 2.0f, 0.5f), {SdfPose{.position = {1.5f, 0.0f, -2.0f}}}, {0.5f});
    const RenderScene scene = scene_or_fail(world);
    Camera camera;
    camera.position = glm::vec3(1.5f, 0.0f, 0.0f);  // in the slab, clear of the hole

    const Frame raster = render_frame(scene, camera, DrawMode::shaded);
    const Frame reference = render_frame(scene, camera, DrawMode::raymarch);
    uint32_t blue = 0;
    EXPECT_LT(shows_mismatch(scene, raster, reference, &blue), 0.01);
    EXPECT_GT(blue, 0u) << "the sphere behind the slab shows";
}

// Pure: the same scene, camera and options give the same bytes, with
// shadows and overlays on.
TEST(CsgRaster, TheSameFrameTwiceIsByteIdentical) {
    const WorldDesc world = slab_and_spheres(glm::vec3(1.0f, 1.0f, 0.1f),
                                             {SdfPose{.position = {0.8f, 0.0f, 0.6f}}}, {0.3f});
    const RenderScene scene = scene_or_fail(world);
    Camera camera;
    camera.position = glm::vec3(0.5f, 1.0f, 3.5f);
    const Frame first = render_frame(scene, camera, DrawMode::shaded, /*overlays=*/true, /*shadows=*/true);
    const Frame second = render_frame(scene, camera, DrawMode::shaded, /*overlays=*/true, /*shadows=*/true);
    EXPECT_EQ(first.storage, second.storage);
}


// L6: a subtree with no program, or pointing past the statics, would vanish
// from the frame in silence. render() refuses both before drawing anything.
TEST(CsgRaster, RefusesAnEmptyOrUnattachedSubtree) {
    const WorldDesc world = slab_and_spheres(glm::vec3(1.0f, 1.0f, 0.1f), {}, {});
    const RenderScene good = scene_or_fail(world);
    ASSERT_EQ(good.csg_subtrees.size(), 1u);
    std::vector<uint8_t> storage(static_cast<size_t>(kWidth) * kHeight * 4u, 0u);
    RenderTarget target{.pixels = std::span<uint8_t>(storage), .width = kWidth, .height = kHeight,
                        .stride = kWidth * 4u, .format = PixelFormat::bgrx8};
    for (const DrawMode mode : {DrawMode::shaded, DrawMode::wireframe}) {
        SCOPED_TRACE(mode == DrawMode::shaded ? "shaded" : "wireframe");
        RenderOptions options;
        options.mode = mode;
        RenderScene empty = good;
        empty.csg_subtrees[0].program = spade::SdfProgram{};
        const Result<void> a = spade::render::render(empty, Camera{}, options, target);
        ASSERT_FALSE(a.has_value()) << "an empty subtree program must be refused";
        EXPECT_EQ(a.error().code, spade::Code::invalid_argument);

        RenderScene unattached = good;
        unattached.csg_subtrees[0].draw_item = static_cast<uint32_t>(unattached.statics.size());
        const Result<void> b = spade::render::render(unattached, Camera{}, options, target);
        ASSERT_FALSE(b.has_value()) << "a subtree whose draw item is past the statics must be refused";
        EXPECT_EQ(b.error().code, spade::Code::invalid_argument);
    }
}

}  // namespace
