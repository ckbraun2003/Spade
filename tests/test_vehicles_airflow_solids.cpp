// vehicles::AirflowSolids (vehicles/airflow_solids.hpp): a model's posed
// parts as the airflow grid's solids, the adapter a live run hands
// set_solids(). Here the reference quad -- the library's
// reference_quad.model.yaml, compiled from the same fixture -- held at hover
// at the D1 film's resolutions (Rendering's ask, 2026-10-22: the case-quad
// film carries the airframe in the flow at dx = 4 mm).
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include <glm/gtc/quaternion.hpp>

#include "reference_airframe.hpp"
#include "physics/airflow/solids.hpp"
#include "scene/model_file.hpp"
#include "state/layout.hpp"
#include "vehicles/airflow_solids.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/model_identity.hpp"
#include "vehicles/part_pose.hpp"

namespace {

using namespace spade::physics::airflow;

struct QuadBuild {
    spade::Result<void> ok;
    SolidField field;
};

// The quad at rest at the world origin, upright (rotor axes +Y), in a grid
// reaching 0.22 m out along x and z and 0.08 m along y (its 0.372 m span and
// the parts' 2 dx reach).
[[nodiscard]] QuadBuild build_quad(float dx) {
    static const spade::vehicles::ModelType quad =
        spade::vehicles::compile_airframe(spade::vehicles::fixtures::reference_quad()).value().model;
    const glm::vec3 reach(0.22f, 0.08f, 0.22f);
    const auto cells = [dx](float metres) { return static_cast<uint32_t>(std::ceil(2.0f * metres / dx)); };
    const GridShape shape{cells(reach.x), cells(reach.y), cells(reach.z), dx};
    spade::BodyState body{};
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);  // upright; a zero quaternion is refused
    const std::array<float, 4> omega{};
    const std::array<float, 4> psi{};
    const auto posed = spade::vehicles::posed_parts(quad, body, omega, psi, spade::vehicles::PartFilter::rigid_only);
    EXPECT_TRUE(posed.has_value()) << (posed ? "" : posed.error().context);
    QuadBuild out{{}, *make_solid_field(shape)};
    if (!posed) return out;
    const spade::vehicles::AirflowSolids solids(quad, *posed, body.pos - reach);
    const auto parts = solids.parts();
    EXPECT_TRUE(parts.has_value()) << (parts ? "" : parts.error().context);
    if (!parts) return out;
    out.ok = build_solids(out.field, shape, {}, *parts, solids.sampler());
    std::printf("reference quad at dx %.1f mm (%ux%ux%u): %s\n", 1000.0 * dx, shape.nx, shape.ny, shape.nz,
                out.ok ? out.field.run_line.c_str() : out.ok.error().context.c_str());
    return out;
}

// At 4 mm the quad is ten clean walls: its 12 mm arms are 3 cells across,
// and the measured safeguard finds no leak and resheets nothing (measured
// 2026-10-22, MSVC 14.43 release: "solids: 10 parts, 10 walls, 0 sheets").
TEST(VehiclesAirflowSolids, TheReferenceQuadAtHoverIsTenCleanWallsAt4Millimetres) {
    const QuadBuild b = build_quad(0.004f);
    ASSERT_TRUE(b.ok.has_value()) << b.ok.error().context;
    ASSERT_EQ(b.field.classes.size(), 10u);
    for (const SolidClass& c : b.field.classes) EXPECT_EQ(c.role, SolidRole::wall);
    EXPECT_EQ(b.field.run_line.find("leaks"), std::string::npos) << b.field.run_line;
    EXPECT_EQ(b.field.run_line.find(": sheet ("), std::string::npos) << b.field.run_line;
    uint32_t closed_faces = 0;
    for (uint32_t c = 0; c < 3; ++c) {
        for (const float t : b.field.theta[c]) closed_faces += closed(t) ? 1u : 0u;
    }
    EXPECT_GT(closed_faces, 0u);
}

// Coarser, the arms near the classifier's 2 dx edge leak at their edges and
// the run says so by part (L6), still all walls. Measured 2026-10-22: at 5 mm
// (2.4 cells) parts[3] and [4] announce 2 runs of 0.000 dx^2 each (printed,
// not pinned: a last-bit case); at 6 mm (D1's nest, 2 cells) all four arms
// announce, parts[3] and [4] with 0.567 dx^2, 1.55% of their largest face
// (pinned). Mutation: the leak announcement dropped.
TEST(VehiclesAirflowSolids, TheReferenceQuadsArmsAnnounceTheirEdgeLeaksAt6Millimetres) {
    for (const float dx : {0.005f, 0.006f}) {
        const QuadBuild b = build_quad(dx);
        ASSERT_TRUE(b.ok.has_value()) << dx << ": " << b.ok.error().context;
        for (const SolidClass& c : b.field.classes) EXPECT_EQ(c.role, SolidRole::wall) << dx;
    }
    const QuadBuild six = build_quad(0.006f);
    ASSERT_TRUE(six.ok.has_value());
    EXPECT_NE(six.field.run_line.find("parts[3]: leaks at an edge"), std::string::npos) << six.field.run_line;
    EXPECT_NE(six.field.run_line.find("parts[4]: leaks at an edge"), std::string::npos) << six.field.run_line;
}

// Coarser than 6 mm the arms (12 mm across in two extents) are wires,
// refused by name with the dx they need (L6, the classifier's rule).
TEST(VehiclesAirflowSolids, TheReferenceQuadsArmsAreRefusedAsWiresAt7Millimetres) {
    const QuadBuild b = build_quad(0.007f);
    ASSERT_FALSE(b.ok.has_value());
    const std::string& why = b.ok.error().context;
    EXPECT_NE(why.find("parts[1]"), std::string::npos) << why;
    EXPECT_NE(why.find("two extents"), std::string::npos) << why;
}

}  // namespace
// ---------------------------------------------------------------------------
// N15, DRAWN ONLY (Physics' ruling, 2026-10-22; the lead's grant to
// Interface, Physics reviewing). A part whose role is kDrawnOnlyRole is drawn
// but is not a solid: the adapter leaves it out of parts() and the sampler,
// and drawn_only() names it, for the run line. Marking a part is the user's
// modelling choice; the solver never drops a part of its own accord ("refused,
// never dropped"). The role is the one mark: PartDesc gains no field.
// ---------------------------------------------------------------------------

namespace {

[[nodiscard]] const spade::vehicles::ModelType& reference_quad_model() {
    static const spade::vehicles::ModelType quad =
        spade::vehicles::compile_airframe(spade::vehicles::fixtures::reference_quad()).value().model;
    return quad;
}

[[nodiscard]] spade::Result<spade::vehicles::PosedParts> posed_upright(const spade::vehicles::ModelType& m) {
    spade::BodyState body{};
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const std::vector<float> still(m.rotors.size(), 0.0f);
    return spade::vehicles::posed_parts(m, body, still, still, spade::vehicles::PartFilter::rigid_only);
}

}  // namespace

TEST(VehiclesAirflowSolids, APartMarkedDrawnOnlyIsLeftOutOfTheSolidsAndNamedForTheRunLine) {
    const spade::vehicles::ModelType& plain = reference_quad_model();
    const auto plain_posed = posed_upright(plain);
    ASSERT_TRUE(plain_posed.has_value()) << plain_posed.error().context;
    const std::size_t marked_part = plain_posed->parts.back().part;
    spade::vehicles::ModelType marked = plain;
    marked.parts[marked_part].role = std::string(spade::vehicles::kDrawnOnlyRole);
    const auto marked_posed = posed_upright(marked);
    ASSERT_TRUE(marked_posed.has_value()) << marked_posed.error().context;

    const spade::vehicles::AirflowSolids all(plain, *plain_posed, glm::vec3(-0.22f, -0.08f, -0.22f));
    const spade::vehicles::AirflowSolids some(marked, *marked_posed, glm::vec3(-0.22f, -0.08f, -0.22f));
    const auto all_parts = all.parts();
    const auto some_parts = some.parts();
    ASSERT_TRUE(all_parts.has_value() && some_parts.has_value());
    EXPECT_TRUE(all.drawn_only().empty());
    ASSERT_EQ(some_parts->size() + 1, all_parts->size());
    ASSERT_EQ(some.drawn_only().size(), 1u);
    EXPECT_EQ(some.drawn_only().front(), plain.parts[marked_part].name);
    for (const SolidPart& s : *some_parts) EXPECT_NE(s.name, plain.parts[marked_part].name);

    // The grid builds from the rest, the sampler indexing them alone.
    const float dx = 0.004f;
    const auto cells = [dx](float metres) { return static_cast<uint32_t>(std::ceil(2.0f * metres / dx)); };
    const GridShape shape{cells(0.22f), cells(0.08f), cells(0.22f), dx};
    SolidField field = *make_solid_field(shape);
    const spade::Result<void> built = build_solids(field, shape, {}, *some_parts, some.sampler());
    ASSERT_TRUE(built.has_value()) << built.error().context;
    EXPECT_EQ(field.classes.size(), some_parts->size());
}

// The mark is a role, which model_identity() never folds (it never folds
// parts): the model's identity stays, and parts_identity() moves, as any part
// edit moves it. No library model carries the mark, so every one keeps every
// part a solid.
TEST(VehiclesAirflowSolids, TheMarkLeavesTheModelIdentityAndNoLibraryModelCarriesIt) {
    const spade::vehicles::ModelType& plain = reference_quad_model();
    spade::vehicles::ModelType marked = plain;
    marked.parts.front().role = std::string(spade::vehicles::kDrawnOnlyRole);
    EXPECT_EQ(spade::vehicles::model_identity(marked), spade::vehicles::model_identity(plain));
    EXPECT_NE(spade::vehicles::parts_identity(marked), spade::vehicles::parts_identity(plain));

    std::size_t models = 0;
    for (const auto& e : std::filesystem::recursive_directory_iterator(std::filesystem::path(SPADE_ASSETS_DIR) /
                                                                      "library")) {
        if (!e.is_regular_file() || !e.path().filename().string().ends_with(".model.yaml")) continue;
        ++models;
        const spade::Result<spade::scene::ModelFile> file = spade::scene::load_model_file(e.path());
        ASSERT_TRUE(file.has_value()) << e.path().string() << ": " << file.error().context;
        const auto posed = posed_upright(file->model);
        ASSERT_TRUE(posed.has_value()) << e.path().string() << ": " << posed.error().context;
        const spade::vehicles::AirflowSolids solids(file->model, *posed, glm::vec3(0.0f));
        const auto parts = solids.parts();
        ASSERT_TRUE(parts.has_value()) << e.path().string() << ": " << parts.error().context;
        EXPECT_TRUE(solids.drawn_only().empty()) << e.path().string();
        EXPECT_EQ(parts->size(), posed->parts.size()) << e.path().string();
    }
    EXPECT_GT(models, 0u);
}
