// The library drones as airflow solids (L7b2 of the realistic drones plan,
// docs/design/core/plans/2026-10-26-l7-realistic-drones-draft.md; Physics
// reviews this file). Each drone's rigid parts -- its body, arms and bells
// (meshes) and its ESC and pack (analytic) -- posed upright at rest the way
// the airflow poses a model (PartFilter::rigid_only: no props), and gridded
// as Physics' build_quad() grids the reference quad
// (test_vehicles_airflow_solids.cpp), at the D1 film's 4 mm. The Crazyflie
// keeps its true size (the lead's Q5 (a)): at 4 mm its 5 mm arms are refused
// by name, with the cell size they need.
//
// A part wholly inside another -- an ESC in the stack, a pack inside the
// shell -- closes no face, and the classifier refuses it as a leak today. The
// lead's ruling (a), 2026-10-27: such a part is covered, not leaking (it adds
// nothing to the boundary; its mass stays in the composite), Physics' change
// in physics/airflow/solids.cpp. Until it merges each drone names what the
// classifier says of its enclosed part -- a leak it refuses, or a sheet it
// measures; when it merges, kEnclosedPartsAreCovered turns true in the same
// change and every drone is clean walls.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <string_view>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "library_drones.hpp"
#include "physics/airflow/solids.hpp"
#include "vehicles/airflow_solids.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/part_pose.hpp"

namespace {

namespace ld = spade::test::library_drones;
using namespace spade::physics::airflow;

struct DroneSolids {
    spade::Result<void> ok;
    SolidField field;
    std::size_t rigid_parts = 0;  // the model's parts that are not on a rotor
};

// The drone `stem`, built from the committed library and compiled, posed
// upright at rest at the origin with rigid_only, in a grid holding its posed
// parts' boxes and 2 dx more on every side (the classifier's reach), plus a
// cell for the rounding, and a third of a cell more below. The box's corner
// is a part's own face, so a part a whole or half number of cells across put
// its far face on a plane of cell centres or faces by construction: the
// Phantom's and the Iris's 50 mm bodies, 12.5 cells at 4 mm, whose tops on
// the cell centres the classifier refused, 500-odd runs leaking, once the
// meshes kept the design axes exactly (L7b2, reported to Physics).
[[nodiscard]] DroneSolids build_drone_solids(std::string_view stem, float dx) {
    DroneSolids out{{}, {}, 0};
    const spade::Result<ld::Committed> lib = ld::committed_library();
    EXPECT_TRUE(lib.has_value()) << (lib ? "" : lib.error().context);
    if (!lib) return out;
    const spade::Result<ld::BuiltDrone> drone = ld::build_drone(lib->library, lib->registry, stem);
    EXPECT_TRUE(drone.has_value()) << (drone ? "" : drone.error().context);
    if (!drone) return out;
    const auto compiled = spade::vehicles::compile_airframe(drone->spec);
    EXPECT_TRUE(compiled.has_value()) << (compiled ? "" : compiled.error().context);
    if (!compiled) return out;
    const spade::vehicles::ModelType& model = compiled->model;
    for (const spade::vehicles::PartDesc& p : model.parts) {
        out.rigid_parts += p.attach == spade::vehicles::PartAttach::body ? 1u : 0u;
    }

    spade::BodyState body{};
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);  // upright; a zero quaternion is refused
    const auto posed = spade::vehicles::posed_parts(model, body, {}, {}, spade::vehicles::PartFilter::rigid_only);
    EXPECT_TRUE(posed.has_value()) << (posed ? "" : posed.error().context);
    if (!posed) return out;

    // The posed boxes, world frame (an origin of 0), then the grid around them.
    glm::vec3 lo(std::numeric_limits<float>::max());
    glm::vec3 hi(std::numeric_limits<float>::lowest());
    {
        const spade::vehicles::AirflowSolids world(model, *posed, glm::vec3(0.0f));
        const auto parts = world.parts();
        EXPECT_TRUE(parts.has_value()) << (parts ? "" : parts.error().context);
        if (!parts) return out;
        for (const SolidPart& p : *parts) {
            lo = glm::min(lo, p.box_min);
            hi = glm::max(hi, p.box_max);
        }
    }
    const glm::vec3 origin = lo - glm::vec3(3.0f * dx + dx / 3.0f);
    const glm::vec3 size = hi - lo + glm::vec3(6.0f * dx + dx / 3.0f);
    const auto cells = [dx](float metres) { return static_cast<uint32_t>(std::ceil(metres / dx)); };
    const GridShape shape{cells(size.x), cells(size.y), cells(size.z), dx};
    const spade::Result<SolidField> field = make_solid_field(shape);
    EXPECT_TRUE(field.has_value()) << (field ? "" : field.error().context);
    if (!field) return out;
    out.field = *field;

    const spade::vehicles::AirflowSolids solids(model, *posed, origin);
    const auto parts = solids.parts();
    EXPECT_TRUE(parts.has_value()) << (parts ? "" : parts.error().context);
    if (!parts) return out;
    out.ok = build_solids(out.field, shape, {}, *parts, solids.sampler());
    std::printf("%.*s at dx %.1f mm (%ux%ux%u): %s\n", static_cast<int>(stem.size()), stem.data(), 1000.0 * dx,
                shape.nx, shape.ny, shape.nz, out.ok ? out.field.run_line.c_str() : out.ok.error().context.c_str());
    return out;
}

}  // namespace

// Ruling (a) in solids.cpp: an enclosed part is covered, not a leak.
constexpr bool kEnclosedPartsAreCovered = false;

// The five drones built to the solids rule (each part 8 mm or more across),
// each with what the classifier says of its enclosed part until (a): a pack
// or an ESC that closes no face is refused as a leak, or, crossed thin, is
// measured a sheet.
struct Enclosed {
    std::string_view stem;
    std::string_view today;  // in the refusal, or in the run line
};
constexpr std::array<Enclosed, 5> kDrones{{
    {"phantom3_class", "part 'battery' leaks: it closes no face"},
    {"iris_plus_class", "part 'battery' leaks: it closes no face"},
    {"fpv_5inch", "part 'esc' leaks: it closes no face"},
    {"basic_quad_6inch", "; esc: sheet (measured"},
    {"basic_quad_competition", "; esc: sheet (measured"},
}};

// At 4 mm every rigid part of the five drones is a wall -- no sheet, one
// class per rigid part, any leak at an edge within the classifier's own rule
// (a note, as the reference quad's at 6 mm; the meshed arms at 45 degrees
// leave a few) -- once an enclosed part is covered; until then each names its
// enclosed part, which closes no face.
TEST(LibraryDroneSolids, AtFourMillimetresEachDroneIsWallsOrNamesItsEnclosedPart) {
    for (const auto& [stem, today] : kDrones) {
        SCOPED_TRACE(stem);
        const DroneSolids b = build_drone_solids(stem, 0.004f);
        if (!kEnclosedPartsAreCovered) {
            const std::string& said = b.ok ? b.field.run_line : b.ok.error().context;
            EXPECT_NE(said.find(today), std::string::npos) << said;
            continue;
        }
        ASSERT_TRUE(b.ok.has_value()) << b.ok.error().context;
        EXPECT_EQ(b.field.classes.size(), b.rigid_parts);
        for (const SolidClass& c : b.field.classes) EXPECT_EQ(c.role, SolidRole::wall);
        EXPECT_EQ(b.field.run_line.find(": sheet ("), std::string::npos) << b.field.run_line;
    }
}

// The Crazyflie keeps its true size (Q5 (a)), so at 4 mm its first arm --
// 5 mm across, below 2 dx in two extents -- is refused by name, with the
// cell size it needs: dx <= 2.5 mm.
TEST(LibraryDroneSolids, TheCrazyflieNamesTheCellSizeItNeeds) {
    const DroneSolids b = build_drone_solids("crazyflie_class", 0.004f);
    ASSERT_FALSE(b.ok.has_value()) << b.field.run_line;
    EXPECT_EQ(b.ok.error().code, spade::Code::invalid_argument);
    const std::string& why = b.ok.error().context;
    EXPECT_NE(why.find("part 'arm[0]'"), std::string::npos) << why;
    EXPECT_NE(why.find("needs dx <= 2.5 mm"), std::string::npos) << why;
}
