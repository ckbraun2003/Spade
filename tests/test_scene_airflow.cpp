// test_scene_airflow.cpp -- scene version 5's airflow regions (N5b): their
// text, their rules (Physics' check_grid() for the grid, by name for the
// rest) and their identity. scene/scene_file.hpp has the contract; the
// editor end-goal design's §4 has Physics' semantics.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "core/error.hpp"
#include "core/rng.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"
#include "scene/scene_file.hpp"

namespace {

using spade::Code;
using spade::physics::airflow::FaceBc;
using spade::physics::airflow::GridShape;
using spade::scene::AirflowCoupling;
using spade::scene::AirflowKind;
using spade::scene::SceneAirflowRegion;
using spade::scene::SceneDesc;
namespace af = spade::physics::airflow;

bool contains(const std::string& haystack, std::string_view needle) {
    return haystack.find(needle) != std::string::npos;
}

template <class T>
std::string why(const spade::Result<T>& r) {
    return r ? std::string("<succeeded>") : r.error().context;
}

std::string with(std::string text, std::string_view from, std::string_view to) {
    const std::size_t at = text.find(from);
    EXPECT_NE(at, std::string::npos) << "fixture text has no '" << from << "'";
    if (at != std::string::npos) text.replace(at, from.size(), to);
    return text;
}

// gate_run: two vehicles, quad_1 and quad_2, and four assets.
SceneDesc gate_run() {
    const auto scene = spade::scene::load_scene_file(std::filesystem::path(SPADE_GOLDEN_DIR) /
                                                     "scenes/gate_run.scene.yaml");
    EXPECT_TRUE(scene.has_value()) << why(scene);
    return scene ? *scene : SceneDesc{};
}

// A drone box around quad_1, standing on the floor at y = 0.
SceneAirflowRegion drone_box() {
    SceneAirflowRegion r;
    r.name = "drone box";
    r.kind = AirflowKind::follow;
    r.follows = "quad_1";
    r.cells = GridShape{32, 24, 32, 0.02f};
    r.faces.ambient = glm::vec3(1.5f, 0.0f, 0.0f);
    r.density = 1.225f;
    r.kinematic_viscosity = 1.5e-5f;
    r.floor = 0.0f;
    return r;
}

// A fixed tunnel: open ends along x, a still floor, a moving lid, periodic
// along z.
SceneAirflowRegion tunnel() {
    SceneAirflowRegion r;
    r.name = "tunnel";
    r.kind = AirflowKind::fixed;
    r.min_corner = glm::vec3(-2.0f, 0.0f, -12.0f);
    r.cells = GridShape{64, 32, 32, 0.05f};
    r.faces.face = {FaceBc::open, FaceBc::open, FaceBc::wall, FaceBc::wall, FaceBc::periodic, FaceBc::periodic};
    r.faces.wall_velocity[af::kYPlus] = glm::vec3(0.5f, 0.0f, 0.0f);
    r.faces.ambient = glm::vec3(3.0f, 0.0f, 0.0f);
    r.density = 1.2f;
    r.kinematic_viscosity = 1.5e-5f;
    return r;
}

// The two items, as the writer must write them (property 1, canonical).
constexpr std::string_view kDroneBoxItem =
    "  - name: \"drone box\"\n"
    "    kind: follow\n"
    "    follows: \"quad_1\"\n"
    "    cells: [32, 24, 32]\n"
    "    dx: 0.0199999996\n"
    "    wind: [1.5, 0, 0]\n"
    "    fluid: {density: 1.22500002, kinematic_viscosity: 1.49999996e-05}\n"
    "    coupling: one_way\n"
    "    floor: {height: 0}\n";
constexpr std::string_view kTunnelItem =
    "  - name: \"tunnel\"\n"
    "    kind: fixed\n"
    "    min_corner: [-2, 0, -12]\n"
    "    cells: [64, 32, 32]\n"
    "    dx: 0.0500000007\n"
    "    faces: {x_minus: open, x_plus: open, y_minus: {wall: [0, 0, 0]}, y_plus: {wall: [0.5, 0, 0]}, "
    "z_minus: periodic, z_plus: periodic}\n"
    "    wind: [3, 0, 0]\n"
    "    fluid: {density: 1.20000005, kinematic_viscosity: 1.49999996e-05}\n"
    "    coupling: one_way\n";

SceneDesc with_regions() {
    SceneDesc s = gate_run();
    s.airflow = {drone_box(), tunnel()};
    return s;
}

void expect_same_region(const SceneAirflowRegion& a, const SceneAirflowRegion& b) {
    EXPECT_EQ(a.name, b.name);
    EXPECT_EQ(a.kind, b.kind) << a.name;
    EXPECT_EQ(a.follows, b.follows) << a.name;
    EXPECT_EQ(a.min_corner, b.min_corner) << a.name;
    EXPECT_EQ(a.cells.nx, b.cells.nx) << a.name;
    EXPECT_EQ(a.cells.ny, b.cells.ny) << a.name;
    EXPECT_EQ(a.cells.nz, b.cells.nz) << a.name;
    EXPECT_EQ(a.cells.dx, b.cells.dx) << a.name;
    EXPECT_EQ(a.faces.face, b.faces.face) << a.name;
    EXPECT_EQ(a.faces.wall_velocity, b.faces.wall_velocity) << a.name;
    EXPECT_EQ(a.faces.ambient, b.faces.ambient) << a.name;
    EXPECT_EQ(a.density, b.density) << a.name;
    EXPECT_EQ(a.kinematic_viscosity, b.kinematic_viscosity) << a.name;
    EXPECT_EQ(a.coupling, b.coupling) << a.name;
    EXPECT_EQ(a.floor, b.floor) << a.name;
}

}  // namespace

TEST(SceneAirflow, RegionsRoundTripThroughTheirText) {
    const SceneDesc scene = with_regions();
    const auto text = spade::scene::scene_to_yaml(scene);
    ASSERT_TRUE(text.has_value()) << why(text);
    EXPECT_TRUE(contains(*text, "\nscene_version: 5\n")) << *text;
    EXPECT_TRUE(contains(*text, "\nairflow:\n" + std::string(kDroneBoxItem) + std::string(kTunnelItem)))
        << *text;
    // After the vehicles, before spare.
    EXPECT_LT(text->find("\nvehicles:\n"), text->find("\nairflow:\n"));
    EXPECT_LT(text->find("\nairflow:\n"), text->find("\nspare:"));

    const auto back = spade::scene::scene_from_yaml(*text);
    ASSERT_TRUE(back.has_value()) << why(back);
    ASSERT_EQ(back->airflow.size(), 2u);
    expect_same_region(back->airflow[0], scene.airflow[0]);
    expect_same_region(back->airflow[1], scene.airflow[1]);
    const auto again = spade::scene::scene_to_yaml(*back);
    ASSERT_TRUE(again.has_value()) << why(again);
    EXPECT_EQ(*again, *text);
}

TEST(SceneAirflow, ASceneWithoutRegionsKeepsItsVersionAndItsText) {
    // gate_run is version 1, and stays so byte for byte; a version-5 file
    // with no region reads, and is written as the oldest version that
    // carries it.
    const SceneDesc scene = gate_run();
    const auto text = spade::scene::scene_to_yaml(scene);
    ASSERT_TRUE(text.has_value()) << why(text);
    EXPECT_TRUE(contains(*text, "\nscene_version: 1\n")) << *text;
    EXPECT_FALSE(contains(*text, "airflow")) << *text;
    const auto five = spade::scene::scene_from_yaml(with(*text, "\nscene_version: 1\n", "\nscene_version: 5\n"));
    ASSERT_TRUE(five.has_value()) << why(five);
    EXPECT_TRUE(five->airflow.empty());
    const auto rewritten = spade::scene::scene_to_yaml(*five);
    ASSERT_TRUE(rewritten.has_value()) << why(rewritten);
    EXPECT_EQ(*rewritten, *text);
}

TEST(SceneAirflow, AirflowIsAnUnknownKeyBeforeVersion5) {
    const auto text = spade::scene::scene_to_yaml(with_regions());
    ASSERT_TRUE(text.has_value()) << why(text);
    const auto four = spade::scene::scene_from_yaml(with(*text, "\nscene_version: 5\n", "\nscene_version: 4\n"));
    ASSERT_FALSE(four.has_value());
    EXPECT_EQ(four.error().code, Code::invalid_argument);
    EXPECT_TRUE(contains(why(four), "unknown key 'airflow' in scene -- scene schema v4 does not define it"))
        << why(four);
}

TEST(SceneAirflow, TheGridIsPhysicsCheckGrid) {
    // Each grid refusal is check_grid()'s own (TD-9): the same code, and its
    // message after the region's name. A follow region's grid is checked
    // with open faces and its wind.
    struct Case {
        std::string label;
        std::function<void(SceneAirflowRegion&)> edit;
        std::size_t region;  // 0 the drone box, 1 the tunnel
    };
    const std::vector<Case> cases = {
        {"no cells", [](SceneAirflowRegion& r) { r.cells.ny = 0; }, 1},
        {"dx zero", [](SceneAirflowRegion& r) { r.cells.dx = 0.0f; }, 0},
        {"one periodic face", [](SceneAirflowRegion& r) { r.faces.face[af::kZPlus] = FaceBc::wall; }, 1},
        {"odd periodic axis", [](SceneAirflowRegion& r) { r.cells.nz = 31; }, 1},
        {"a closed box blowing in",
         [](SceneAirflowRegion& r) {
             r.faces.face = {FaceBc::wall, FaceBc::wall, FaceBc::wall, FaceBc::wall, FaceBc::wall, FaceBc::wall};
             r.faces.wall_velocity[af::kXMinus] = glm::vec3(1.0f, 0.0f, 0.0f);
         },
         1},
        {"past the byte cap", [](SceneAirflowRegion& r) { r.cells = GridShape{1024, 1024, 1024, 0.01f}; }, 0},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.label);
        SceneDesc scene = with_regions();
        SceneAirflowRegion& r = scene.airflow[c.region];
        c.edit(r);
        af::DomainBc bc = r.faces;
        if (r.kind == AirflowKind::follow) {
            bc = af::DomainBc{};
            bc.ambient = r.faces.ambient;
        }
        const spade::Result<void> grid = af::check_grid(r.cells, bc, true);
        ASSERT_FALSE(grid.has_value()) << "the case must break the grid";
        const spade::Result<void> refused = spade::scene::validate_scene(scene);
        ASSERT_FALSE(refused.has_value());
        EXPECT_EQ(refused.error().code, grid.error().code) << why(refused);
        EXPECT_EQ(why(refused), "airflow region '" + r.name + "': " + grid.error().context);
    }
}

TEST(SceneAirflow, RegionRulesAreRefusedByName) {
    static constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
    static constexpr float kInf = std::numeric_limits<float>::infinity();
    struct Case {
        std::function<void(SceneDesc&)> edit;
        std::string needle;
    };
    const std::vector<Case> cases = {
        {[](SceneDesc& s) { s.airflow[0].name.clear(); }, "airflow region 0 has no name"},
        {[](SceneDesc& s) { s.airflow[1].name = "drone box"; }, "duplicate airflow region name 'drone box'"},
        {[](SceneDesc& s) { s.airflow[0].kind = static_cast<AirflowKind>(7); },
         "airflow region 'drone box': kind 7 is not follow or fixed"},
        {[](SceneDesc& s) { s.airflow[1].coupling = AirflowCoupling::two_way; },
         "airflow region 'tunnel': two_way coupling waits for Physics' D4; a region couples one_way"},
        {[](SceneDesc& s) { s.airflow[1].coupling = static_cast<AirflowCoupling>(9); },
         "airflow region 'tunnel': coupling 9 is not one_way or two_way"},
        {[](SceneDesc& s) { s.airflow[0].follows = "nobody"; },
         "airflow region 'drone box': follows 'nobody', which is no vehicle of the scene"},
        {[](SceneDesc& s) {
             SceneAirflowRegion second = drone_box();
             second.name = "second box";
             s.airflow.push_back(second);
         },
         "airflow region 'second box': vehicle 'quad_1' is followed by 'drone box' already; a vehicle has one "
         "follow region"},
        {[](SceneDesc& s) { s.airflow[0].min_corner = glm::vec3(0.0f, 1.0f, 0.0f); },
         "airflow region 'drone box': a follow region's box goes with its vehicle; its min_corner must be 0 0 0"},
        {[](SceneDesc& s) { s.airflow[0].faces.face[af::kYMinus] = FaceBc::wall; },
         "airflow region 'drone box': a follow region's faces come from its flight (Physics' resolve()); they "
         "must all be open, with no wall velocity"},
        {[](SceneDesc& s) { s.airflow[0].faces.wall_velocity[af::kXPlus] = glm::vec3(0.0f, 1.0f, 0.0f); },
         "a follow region's faces come from its flight"},
        {[](SceneDesc& s) { s.airflow[0].floor = kInf; }, "airflow region 'drone box': floor height must be finite"},
        {[](SceneDesc& s) { s.airflow[1].follows = "quad_2"; },
         "airflow region 'tunnel': a fixed region follows no vehicle"},
        {[](SceneDesc& s) { s.airflow[1].floor = 0.0f; },
         "airflow region 'tunnel': a fixed region has no floor; its y_minus face is one, as a wall"},
        {[](SceneDesc& s) { s.airflow[1].min_corner.y = kNan; }, "airflow region 'tunnel': min_corner must be finite"},
        {[](SceneDesc& s) { s.airflow[1].faces.wall_velocity[af::kYPlus].x = kInf; },
         "airflow region 'tunnel': wall velocities must be finite"},
        {[](SceneDesc& s) { s.airflow[0].faces.ambient.z = kNan; }, "airflow region 'drone box': wind must be finite"},
        {[](SceneDesc& s) { s.airflow[1].density = 0.0f; },
         "airflow region 'tunnel': fluid density must be positive and finite"},
        {[](SceneDesc& s) { s.airflow[1].kinematic_viscosity = -1.0f; },
         "airflow region 'tunnel': fluid kinematic_viscosity must be finite and >= 0"},
        {[](SceneDesc& s) {
             SceneAirflowRegion second = tunnel();
             second.name = "tunnel 2";
             second.min_corner.x += 1.0f;  // the tunnel is 3.2 m long
             s.airflow.push_back(second);
         },
         "airflow regions 'tunnel' and 'tunnel 2' overlap; nothing couples two grids"},
    };
    for (std::size_t i = 0; i < cases.size(); ++i) {
        SCOPED_TRACE(i);
        SceneDesc scene = with_regions();
        cases[i].edit(scene);
        const spade::Result<void> refused = spade::scene::validate_scene(scene);
        ASSERT_FALSE(refused.has_value()) << "case " << i;
        EXPECT_EQ(refused.error().code, Code::invalid_argument) << why(refused);
        EXPECT_TRUE(contains(why(refused), cases[i].needle)) << why(refused);
        // The writer refuses what the validator does.
        EXPECT_FALSE(spade::scene::scene_to_yaml(scene).has_value());
    }
    // Two fixed regions that only touch are two grids side by side.
    SceneDesc touching = with_regions();
    SceneAirflowRegion next = tunnel();
    next.name = "tunnel 2";
    next.min_corner.x = -2.0f + 64.0f * 0.05f;
    touching.airflow.push_back(next);
    const spade::Result<void> ok = spade::scene::validate_scene(touching);
    EXPECT_TRUE(ok.has_value()) << why(ok);
}

TEST(SceneAirflow, MalformedRegionTextIsRefusedByKey) {
    const auto base = spade::scene::scene_to_yaml(with_regions());
    ASSERT_TRUE(base.has_value()) << why(base);
    const std::string follows_line = "    follows: \"quad_1\"\n";
    const std::string faces_line =
        "    faces: {x_minus: open, x_plus: open, y_minus: {wall: [0, 0, 0]}, y_plus: {wall: [0.5, 0, 0]}, "
        "z_minus: periodic, z_plus: periodic}\n";
    struct Case {
        std::string text;
        std::string needle;
    };
    const std::vector<Case> cases = {
        {with(*base, follows_line, follows_line + faces_line),
         "scene.airflow[0].faces: a follow region's faces come from its flight (Physics' resolve()); only a fixed "
         "region states them"},
        {with(*base, follows_line, follows_line + "    min_corner: [0, 0, 0]\n"),
         "scene.airflow[0].min_corner: a follow region's box goes with its vehicle"},
        {with(*base, "    min_corner: [-2, 0, -12]\n", "    follows: \"quad_2\"\n    min_corner: [-2, 0, -12]\n"),
         "scene.airflow[1].follows: a fixed region follows no vehicle"},
        {with(*base, "    floor: {height: 0}\n", "    floor: {height: 0}\n    floor: {height: 0}\n"),
         "duplicate key 'floor' in scene.airflow[0]"},
        {with(*base, "    coupling: one_way\n\n", "    coupling: one_way\n    floor: {height: 0}\n\n"),
         "scene.airflow[1].floor: a fixed region has no floor"},
        {with(*base, "    kind: follow\n", "    kind: sideways\n"),
         "scene.airflow[0].kind must be follow or fixed"},
        {with(*base, "    kind: fixed\n", "    kind: [fixed]\n"), "scene.airflow[1].kind must be follow or fixed"},
        {with(*base, "    coupling: one_way\n    floor:", "    coupling: both\n    floor:"),
         "scene.airflow[0].coupling must be one_way or two_way"},
        {with(*base, ", z_plus: periodic}", "}"), "z_plus"},
        {with(*base, "x_plus: open", "x_plus: wall"),
         "scene.airflow[1].faces.x_plus must be open, periodic or {wall: [vx, vy, vz]}"},
        {with(*base, "x_plus: open", "x_plus: {wall: [1, 2]}"), "scene.airflow[1].faces.x_plus.wall"},
        {with(*base, "    cells: [32, 24, 32]\n", "    cells: [32, 24]\n"),
         "scene.airflow[0].cells must be [nx, ny, nz]"},
        {with(*base, "    cells: [32, 24, 32]\n", "    cells: [32, -24, 32]\n"), "scene.airflow[0].cells"},
        {with(*base, "    wind: [1.5, 0, 0]\n", "    wind: [1.5, 0, 0]\n    offset: [0, 1, 0]\n"),
         "unknown key 'offset' in scene.airflow[0] -- scene schema v5 does not define it"},
        {with(*base, "    fluid: {density: 1.22500002, kinematic_viscosity: 1.49999996e-05}\n",
              "    fluid: {density: 1.22500002}\n"),
         "scene.airflow[0].fluid"},
        {with(*base, "    dx: 0.0199999996\n", ""), "scene.airflow[0]"},
    };
    for (std::size_t i = 0; i < cases.size(); ++i) {
        SCOPED_TRACE(i);
        const auto parsed = spade::scene::scene_from_yaml(cases[i].text);
        ASSERT_FALSE(parsed.has_value()) << "case " << i;
        EXPECT_EQ(parsed.error().code, Code::invalid_argument) << why(parsed);
        EXPECT_TRUE(contains(why(parsed), cases[i].needle)) << why(parsed);
    }
}

TEST(SceneAirflow, RegionIdentityIsItsItemsText) {
    // FNV-1a 64 over the tag and the item as the writer writes it: computed
    // here from the expected text, independently of the emitter.
    const auto box = spade::scene::region_identity(drone_box());
    const auto tube = spade::scene::region_identity(tunnel());
    ASSERT_TRUE(box.has_value()) << why(box);
    ASSERT_TRUE(tube.has_value()) << why(tube);
    EXPECT_EQ(*box, spade::rng::fnv1a64("region_identity\n" + std::string(kDroneBoxItem)));
    EXPECT_EQ(*tube, spade::rng::fnv1a64("region_identity\n" + std::string(kTunnelItem)));
    // Every field moves it.
    const std::vector<std::function<void(SceneAirflowRegion&)>> edits = {
        [](SceneAirflowRegion& r) { r.name += "!"; },
        [](SceneAirflowRegion& r) { r.follows = "quad_2"; },
        [](SceneAirflowRegion& r) { r.cells.nx += 2; },
        [](SceneAirflowRegion& r) { r.cells.ny += 2; },
        [](SceneAirflowRegion& r) { r.cells.nz += 2; },
        [](SceneAirflowRegion& r) { r.cells.dx = std::nextafter(r.cells.dx, 1.0f); },
        [](SceneAirflowRegion& r) { r.faces.ambient.y = 0.25f; },
        [](SceneAirflowRegion& r) { r.density = std::nextafter(r.density, 2.0f); },
        [](SceneAirflowRegion& r) { r.kinematic_viscosity = 0.0f; },
        [](SceneAirflowRegion& r) { r.coupling = AirflowCoupling::two_way; },
        [](SceneAirflowRegion& r) { r.floor.reset(); },
        [](SceneAirflowRegion& r) { r.floor = 0.5f; },
        [](SceneAirflowRegion& r) {
            r.kind = AirflowKind::fixed;
            r.follows.clear();
            r.floor.reset();
        },
    };
    for (std::size_t i = 0; i < edits.size(); ++i) {
        SceneAirflowRegion r = drone_box();
        edits[i](r);
        const auto moved = spade::scene::region_identity(r);
        ASSERT_TRUE(moved.has_value()) << why(moved);
        EXPECT_NE(*moved, *box) << "edit " << i;
    }
    // A fixed region's corner and faces move it too.
    const std::vector<std::function<void(SceneAirflowRegion&)>> fixed_edits = {
        [](SceneAirflowRegion& r) { r.min_corner.z = -11.0f; },
        [](SceneAirflowRegion& r) { r.faces.face[af::kXPlus] = FaceBc::wall; },
        [](SceneAirflowRegion& r) { r.faces.wall_velocity[af::kYPlus].z = 0.125f; },
    };
    for (std::size_t i = 0; i < fixed_edits.size(); ++i) {
        SceneAirflowRegion r = tunnel();
        fixed_edits[i](r);
        const auto moved = spade::scene::region_identity(r);
        ASSERT_TRUE(moved.has_value()) << why(moved);
        EXPECT_NE(*moved, *tube) << "fixed edit " << i;
    }
}
