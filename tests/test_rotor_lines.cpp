// Rotors to actuator lines (L86i; physics/airflow/rotor_lines.hpp, the rotor rows' adapter vehicles/rotor_lines.hpp, and the
// propeller part's blade and polar scene/prop_line.hpp): the one rule from a body's rotors to the airflow's lines. Expectations are
// worked by hand from the geometry (TD-4); the drawn blade and the line are held to the same direction.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <fstream>
#include <iterator>
#include <span>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "physics/airflow/blade.hpp"
#include "physics/airflow/line.hpp"
#include "physics/airflow/rotor_lines.hpp"
#include "scene/part_file.hpp"
#include "scene/prop_line.hpp"
#include "state/layout.hpp"
#include "vehicles/rotor.hpp"
#include "vehicles/rotor_lines.hpp"

namespace {

namespace af = spade::physics::airflow;
using spade::Result;

constexpr float kHalfPi = 1.57079632679f;

void expect_near(glm::vec3 got, glm::vec3 want, float tol, const char* what) {
    for (int a = 0; a < 3; ++a) EXPECT_NEAR(got[a], want[a], tol) << what << " [" << a << "]";
}

af::BladeShape two_blade() {
    af::BladeShape b;
    b.blades = 2;
    b.radius = 0.0762f;
    b.stations = {{0.012f, 0.012f, 30.0f}, {0.04f, 0.011f, 20.0f}, {0.0762f, 0.006f, 12.0f}};
    return b;
}

// A body at (1, 2, 3) m turned a quarter turn about world +z (body x -> world y, body y -> world -x), in a grid whose minimum
// corner is at (0.5, 1.5, 2.5). The rotor: hub (0.1, 0.02, 0) in the body, local frame turned a quarter turn about the body's x
// (local +Y -> body +Z, local +X -> body +X).
TEST(AirflowRotorLines, ATurnedBodysRotorGivesItsLineByHand) {
    const af::GridShape grid{32, 32, 32, 0.05f};
    const af::DiscBody body{.com_world = {1.0f, 2.0f, 3.0f},
                            .orient_world = glm::angleAxis(kHalfPi, glm::vec3(0.0f, 0.0f, 1.0f))};
    const af::BladeShape blade = two_blade();
    const af::PolarTable polar = af::estimated_polar();
    const af::LineRotorSpec spec{.hub_body = {0.1f, 0.02f, 0.0f},
                                 .orient_body = glm::angleAxis(kHalfPi, glm::vec3(1.0f, 0.0f, 0.0f)),
                                 .spin_dir = -1.0f,
                                 .omega = 560.0f,
                                 .psi = 0.7f,
                                 .blade = &blade,
                                 .polar = &polar};
    const Result<af::LineRotor> line = af::line_from_rotor(spec, body, {0.5f, 1.5f, 2.5f}, grid);
    ASSERT_TRUE(line.has_value()) << line.error().context;
    // The hub turns to (-0.02, 0.1, 0): the centre is (1 - 0.02, 2 + 0.1, 3) - (0.5, 1.5, 2.5) = (0.48, 0.6, 0.5), the disc's own.
    expect_near(line->centre, {0.48f, 0.6f, 0.5f}, 1.0e-6f, "centre");
    // Local +Y -> body +Z -> world +Z: the line's axis is the thrust's direction, +Z (a disc's axis is its negative).
    expect_near(line->axis, {0.0f, 0.0f, 1.0f}, 1.0e-6f, "axis");
    // Local +X -> body +X -> world +Y: azimuth zero.
    expect_near(line->ref, {0.0f, 1.0f, 0.0f}, 1.0e-6f, "ref");
    EXPECT_EQ(line->spin, -1.0f);
    EXPECT_EQ(line->omega, 560.0f);
    EXPECT_EQ(line->psi, 0.7f);
    EXPECT_EQ(line->blade, &blade);
    EXPECT_EQ(line->polar, &polar);
}

// The drawn blade and the line coincide: posed_parts() turns a rotor's parts by R_y(s psi) about the local +Y, so blade 0 lies along
// R_world R_local R_y(s psi) (+X); the line's first element at the step's mid-azimuth is that direction at s (psi + omega h / 2).
TEST(AirflowRotorLines, TheLineBladeIsWhereTheDrawnBladeIs) {
    const af::GridShape grid{64, 64, 64, 0.01f};
    const af::BladeShape blade = two_blade();
    const af::PolarTable polar = af::estimated_polar();
    for (const float spin : {1.0f, -1.0f}) {
        const af::DiscBody body{.com_world = {0.32f, 0.32f, 0.32f},
                                .orient_world = glm::normalize(glm::angleAxis(0.4f, glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f))))};
        const glm::quat local = glm::normalize(glm::angleAxis(0.3f, glm::normalize(glm::vec3(0.5f, -1.0f, 0.2f))));
        const float psi = 1.1f, omega = 400.0f, h = 0.0005f;
        const af::LineRotorSpec spec{.hub_body = {0.0f, 0.0f, 0.0f}, .orient_body = local, .spin_dir = spin, .omega = omega,
                                     .psi = psi, .blade = &blade, .polar = &polar};
        const Result<af::LineRotor> line = af::line_from_rotor(spec, body, {0.0f, 0.0f, 0.0f}, grid);
        ASSERT_TRUE(line.has_value()) << line.error().context;
        const float theta = spin * (psi + 0.5f * omega * h);
        const glm::vec3 drawn =
            body.orient_world * (local * glm::vec3(std::cos(theta), 0.0f, -std::sin(theta)));  // R_y(theta) +X
        const std::vector<af::LineElement> el = af::line_elements(*line, grid.dx, h, 1);
        ASSERT_FALSE(el.empty());
        const glm::vec3 first = el.front().pos - line->centre;  // blade 0, the first element: along the blade
        expect_near(glm::normalize(first), glm::normalize(drawn), 2.0e-5f, spin > 0.0f ? "spin +1" : "spin -1");
        // The blade moves along s (axis x d): the direction the drawn blade turns.
        expect_near(el.front().motion, glm::normalize(glm::cross(spin * line->axis, glm::normalize(first))), 2.0e-5f, "motion");
    }
}

TEST(AirflowRotorLines, RefusesWhatItCannotMakeALineOfNamingTheField) {
    const af::GridShape grid{32, 32, 32, 0.05f};
    const af::DiscBody body{.com_world = {0.8f, 0.8f, 0.8f}};
    const af::BladeShape blade = two_blade();
    const af::PolarTable polar = af::estimated_polar();
    const af::LineRotorSpec good{.spin_dir = 1.0f, .omega = 100.0f, .psi = 0.1f, .blade = &blade, .polar = &polar};
    ASSERT_TRUE(af::line_from_rotor(good, body, {0.0f, 0.0f, 0.0f}, grid).has_value());
    const auto refused = [&](af::LineRotorSpec s, const af::DiscBody& b, const char* word) {
        const Result<af::LineRotor> r = af::line_from_rotor(s, b, {0.0f, 0.0f, 0.0f}, grid);
        ASSERT_FALSE(r.has_value()) << word;
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << word;
        EXPECT_NE(r.error().context.find(word), std::string::npos) << r.error().context;
    };
    af::LineRotorSpec s = good;
    s.spin_dir = 0.0f;
    refused(s, body, "spin");
    s = good;
    s.omega = -1.0f;
    refused(s, body, "omega");
    s = good;
    s.omega = std::nanf("");
    refused(s, body, "omega");
    s = good;
    s.psi = INFINITY;
    refused(s, body, "psi");
    s = good;
    s.blade = nullptr;
    refused(s, body, "blade");
    s = good;
    s.polar = nullptr;
    refused(s, body, "polar");
    s = good;
    s.orient_body = glm::quat(2.0f, 0.0f, 0.0f, 0.0f);
    refused(s, body, "orientation");
    af::DiscBody b = body;
    b.orient_world = glm::quat(0.0f, 0.0f, 0.0f, 0.0f);
    refused(good, b, "orientation");
    b = body;
    b.com_world.x = INFINITY;
    refused(good, b, "pose");
}

// --- the rotor rows' adapter -------------------------------------------------------------------------------------------

spade::vehicles::RotorRow row_of(glm::vec3 pos, float spin, float omega, bool enabled = true) {
    spade::vehicles::RotorRow r{};
    r.enabled = enabled ? 1u : 0u;
    r.radius = 0.0762f;
    r.local_pos = pos;
    r.spin_dir = spin;
    r.local_orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    r.omega = omega;
    r.thrust_coeff = 2.0087e-6f;
    r.torque_coeff = 2.04075653e-8f;
    return r;
}

TEST(VehicleRotorLines, RowsGiveTheirLinesWhereTheirDiscsWere) {
    const af::GridShape grid{64, 64, 64, 0.01f};
    spade::BodyState body{};
    body.pos = glm::vec3(0.3f, 0.3f, 0.3f);
    body.orient = glm::angleAxis(0.5f, glm::normalize(glm::vec3(0.2f, 1.0f, 0.1f)));
    const std::array<spade::vehicles::RotorRow, 3> rows{row_of({0.11f, 0.024f, 0.0f}, 1.0f, 801.25f),
                                                        row_of({-0.11f, 0.024f, 0.0f}, -1.0f, 790.0f),
                                                        row_of({0.0f, 0.024f, 0.11f}, 1.0f, 640.0f, false)};
    const std::array<float, 3> psi{0.2f, 3.0f, 6.0f};
    const af::BladeShape blade = two_blade();
    const af::PolarTable polar = af::estimated_polar();
    std::array<af::LineRotor, 3> lines{};
    const Result<void> ok = spade::vehicles::rotor_lines(rows, body, psi, {0.0f, 0.0f, 0.0f}, grid, blade, polar, lines);
    ASSERT_TRUE(ok.has_value()) << ok.error().context;
    const af::DiscBody b{.com_world = body.pos, .orient_world = body.orient};
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const af::DiscRotor d{.hub_body = rows[i].local_pos, .orient_body = rows[i].local_orient, .radius = rows[i].radius,
                              .spin_dir = rows[i].spin_dir, .thrust = 1.0f, .torque = 0.01f};
        const Result<af::ActuatorDisc> disc = af::disc_from_rotor(d, b, {0.0f, 0.0f, 0.0f}, grid);
        ASSERT_TRUE(disc.has_value());
        expect_near(lines[i].centre, disc->centre, 1.0e-6f, "centre is the disc's");
        expect_near(lines[i].axis, -disc->axis, 1.0e-6f, "axis is the disc's, reversed");
        EXPECT_EQ(lines[i].spin, rows[i].spin_dir) << i;
        EXPECT_EQ(lines[i].psi, psi[i]) << i;
        EXPECT_EQ(lines[i].blade, &blade) << i;
        EXPECT_EQ(lines[i].polar, &polar) << i;
        EXPECT_NEAR(glm::dot(lines[i].axis, lines[i].ref), 0.0f, 1.0e-6f) << "ref is across the axis";
    }
    EXPECT_EQ(lines[0].omega, 801.25f);
    EXPECT_EQ(lines[1].omega, 790.0f);
    EXPECT_EQ(lines[2].omega, 0.0f) << "a disabled row's blades stand in the air";
}

// The kernel the caller resolved reaches every rotor's line (and the default, 2, is the line's own: nothing changes when it is left out).
TEST(VehicleRotorLines, TheResolvedKernelReachesEveryLine) {
    const af::GridShape grid{64, 64, 64, 0.01f};
    spade::BodyState body{};
    body.pos = glm::vec3(0.3f, 0.3f, 0.3f);
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const std::array<spade::vehicles::RotorRow, 2> rows{row_of({0.11f, 0.024f, 0.0f}, 1.0f, 801.25f), row_of({-0.11f, 0.024f, 0.0f}, -1.0f, 790.0f)};
    const std::array<float, 2> psi{0.2f, 3.0f};
    const af::BladeShape blade = two_blade();
    const af::PolarTable polar = af::estimated_polar();
    std::array<af::LineRotor, 2> lines{};
    ASSERT_TRUE(spade::vehicles::rotor_lines(rows, body, psi, {0.0f, 0.0f, 0.0f}, grid, blade, polar, lines).has_value());
    for (const af::LineRotor& l : lines) EXPECT_EQ(l.eps_cells, 2.0f) << "left out: the line's default";
    ASSERT_TRUE(spade::vehicles::rotor_lines(rows, body, psi, {0.0f, 0.0f, 0.0f}, grid, blade, polar, lines, 2.97f).has_value());
    for (const af::LineRotor& l : lines) EXPECT_EQ(l.eps_cells, 2.97f);
    const auto refused = spade::vehicles::rotor_lines(rows, body, psi, {0.0f, 0.0f, 0.0f}, grid, blade, polar, lines, 3.5f);
    ASSERT_FALSE(refused.has_value());
    EXPECT_NE(refused.error().context.find("eps_cells"), std::string::npos) << refused.error().context;
}

TEST(VehicleRotorLines, RefusesWhatItCannotMapNamingTheRotor) {
    const af::GridShape grid{64, 64, 64, 0.01f};
    spade::BodyState body{};
    body.pos = glm::vec3(0.3f, 0.3f, 0.3f);
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    std::array<spade::vehicles::RotorRow, 2> rows{row_of({0.11f, 0.024f, 0.0f}, 1.0f, 800.0f),
                                                  row_of({-0.11f, 0.024f, 0.0f}, -1.0f, 800.0f)};
    const std::array<float, 2> psi{0.0f, 0.0f};
    const af::BladeShape blade = two_blade();
    const af::PolarTable polar = af::estimated_polar();
    std::array<af::LineRotor, 2> lines{};
    ASSERT_TRUE(spade::vehicles::rotor_lines(rows, body, psi, {0.0f, 0.0f, 0.0f}, grid, blade, polar, lines).has_value());
    // The model's rotor is this propeller: a row whose radius is not the blade's is another prop.
    rows[1].radius = 0.05f;
    const Result<void> radius = spade::vehicles::rotor_lines(rows, body, psi, {0.0f, 0.0f, 0.0f}, grid, blade, polar, lines);
    ASSERT_FALSE(radius.has_value());
    EXPECT_NE(radius.error().context.find("rotor 1"), std::string::npos) << radius.error().context;
    EXPECT_NE(radius.error().context.find("radius"), std::string::npos) << radius.error().context;
    rows[1].radius = 0.0762f;
    rows[1].spin_dir = 0.0f;
    const Result<void> spin = spade::vehicles::rotor_lines(rows, body, psi, {0.0f, 0.0f, 0.0f}, grid, blade, polar, lines);
    ASSERT_FALSE(spin.has_value());
    EXPECT_NE(spin.error().context.find("rotor 1"), std::string::npos) << spin.error().context;
    rows[1].spin_dir = -1.0f;
    const std::array<float, 1> one_psi{0.0f};
    EXPECT_FALSE(spade::vehicles::rotor_lines(rows, body, one_psi, {0.0f, 0.0f, 0.0f}, grid, blade, polar, lines).has_value());
    std::array<af::LineRotor, 1> one_line{};
    EXPECT_FALSE(spade::vehicles::rotor_lines(rows, body, psi, {0.0f, 0.0f, 0.0f}, grid, blade, polar, one_line).has_value());
}

// --- a propeller part as the line's blade and polar ---------------------------------------------------------------------

spade::scene::PropPart library_prop(const char* file) {
    const std::string path = std::string(SPADE_ASSETS_DIR) + "/library/parts/" + file;
    std::ifstream in(path, std::ios::binary);
    EXPECT_TRUE(in.good()) << path;
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    auto part = spade::scene::prop_from_yaml(text);
    EXPECT_TRUE(part.has_value()) << (part ? "" : part.error().context);
    return part ? *part : spade::scene::PropPart{};
}

// phantom3_class's prop: UIUC's APC 10x5 at 9.4 in, with B1's fitted polar.
TEST(ScenePropLine, TheApc10x5IsItsBladeAtItsDiameterWithTheFittedPolar) {
    const spade::scene::PropPart part = library_prop("uiuc_apce_10x5.prop.yaml");
    ASSERT_TRUE(part.blade.has_value());
    const auto prop = spade::scene::line_propeller(part);
    ASSERT_TRUE(prop.has_value()) << prop.error().context;
    const float radius = static_cast<float>(0.5 * part.block.diameter);
    EXPECT_NEAR(radius, 0.1194f, 1.0e-6f);
    EXPECT_EQ(prop->blade.blades, 2u);
    EXPECT_EQ(prop->blade.radius, radius);
    ASSERT_EQ(prop->blade.stations.size(), part.blade->r_over_r.size());
    for (std::size_t i = 0; i < prop->blade.stations.size(); ++i) {
        EXPECT_EQ(prop->blade.stations[i].r, static_cast<float>(part.blade->r_over_r[i] * 0.5 * part.block.diameter)) << i;
        EXPECT_EQ(prop->blade.stations[i].chord, static_cast<float>(part.blade->c_over_r[i] * 0.5 * part.block.diameter)) << i;
        EXPECT_EQ(prop->blade.stations[i].beta_deg, static_cast<float>(part.blade->beta_deg[i])) << i;
    }
    EXPECT_NEAR(prop->blade.stations.front().r, 0.15f * radius, 1.0e-6f) << "the hub's cut-out is the first station";
    EXPECT_NEAR(prop->blade.stations.back().r, radius, 1.0e-6f);
    EXPECT_TRUE(af::check_blade(prop->blade).has_value());
    // B1: alpha_0 -3.064 degrees, drag scale 1.351, through the estimated polar's own builder.
    ASSERT_TRUE(part.blade->alpha_0_deg.has_value() && part.blade->cd_scale.has_value());
    EXPECT_TRUE(prop->fitted);
    const af::PolarTable want = af::estimated_polar(
        {static_cast<float>(*part.blade->alpha_0_deg), static_cast<float>(*part.blade->cd_scale)});
    EXPECT_EQ(prop->polar.key, want.key);
    EXPECT_EQ(prop->polar.cl, want.cl);
    EXPECT_EQ(prop->polar.cd, want.cd);
    EXPECT_NEAR(*part.blade->alpha_0_deg, -3.064, 5.0e-3);
    EXPECT_NEAR(*part.blade->cd_scale, 1.351, 5.0e-3);
}

// A part with no fit takes the estimated polar's defaults, and says so.
TEST(ScenePropLine, APartWithNoFitTakesTheEstimatedPolarAndSaysSo) {
    spade::scene::PropPart part = library_prop("uiuc_apce_10x5.prop.yaml");
    ASSERT_TRUE(part.blade.has_value());
    part.blade->alpha_0_deg.reset();
    part.blade->cd_scale.reset();
    const auto prop = spade::scene::line_propeller(part);
    ASSERT_TRUE(prop.has_value()) << prop.error().context;
    EXPECT_FALSE(prop->fitted);
    const af::PolarTable want = af::estimated_polar();
    EXPECT_EQ(prop->polar.cl, want.cl);
    EXPECT_EQ(prop->polar.cd, want.cd);
}

TEST(ScenePropLine, RefusesAPropWithNoBladeOrNoDiameterNamingTheField) {
    spade::scene::PropPart part = library_prop("uiuc_apce_10x5.prop.yaml");
    spade::scene::PropPart no_blade = part;
    no_blade.blade.reset();
    const auto a = spade::scene::line_propeller(no_blade);
    ASSERT_FALSE(a.has_value());
    EXPECT_EQ(a.error().code, spade::Code::invalid_argument);
    EXPECT_NE(a.error().context.find("blade"), std::string::npos) << a.error().context;
    spade::scene::PropPart no_diameter = part;
    no_diameter.block.diameter = 0.0;
    const auto b = spade::scene::line_propeller(no_diameter);
    ASSERT_FALSE(b.has_value());
    EXPECT_NE(b.error().context.find("diameter"), std::string::npos) << b.error().context;
}

}  // namespace
