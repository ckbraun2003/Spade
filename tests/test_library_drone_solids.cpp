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
// shell -- closes no face, and the classifier refused it as a leak. The
// lead's ruling (a), 2026-10-27: such a part is covered, not leaking (it adds
// nothing to the boundary; its mass stays in the composite), Physics' change
// in physics/airflow/solids.cpp, which turned kEnclosedPartsAreCovered true.
// A covered part is announced on the run line with the parts that cover it;
// one that comes within about 1 dx of its cover's surface is exposed, and
// refused with the faces it would block and how many stay open.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

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
    std::size_t rigid_parts = 0;     // the model's parts that are not on a rotor
    std::vector<std::string> names;  // the solid parts', in order, as field.classes are
};

// How many times `what` occurs in `s`.
[[nodiscard]] std::size_t occurrences(std::string_view s, std::string_view what) {
    std::size_t n = 0;
    for (std::size_t at = s.find(what); at != std::string_view::npos; at = s.find(what, at + what.size())) ++n;
    return n;
}

// The drone `stem`, built from the committed library and compiled, posed
// upright at rest at the origin with rigid_only, in a grid holding its posed
// parts' boxes and 2 dx more on every side (the classifier's reach), plus a
// cell for the rounding, and a third of a cell more below. The box's corner
// is a part's own face, so a part a whole or half number of cells across put
// its far face on a plane of cell centres or faces by construction: the
// Phantom's and the Iris's 50 mm bodies, 12.5 cells at 4 mm, whose tops on
// the cell centres the classifier refused, 500-odd runs leaking, once the
// meshes kept the design axes exactly (L7b2, reported to Physics). Until
// build_solids() answers, `ok` is a refusal of its own, so a drone that is
// never gridded is never taken for one that built.
[[nodiscard]] DroneSolids build_drone_solids(std::string_view stem, float dx) {
    DroneSolids out{std::unexpected(spade::Error{spade::Code::internal, "the drone was never gridded"}), {}, 0, {}};
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
    for (const SolidPart& p : *parts) out.names.push_back(p.name);
    out.ok = build_solids(out.field, shape, {}, *parts, solids.sampler());
    std::printf("%.*s at dx %.1f mm (%ux%ux%u): %s\n", static_cast<int>(stem.size()), stem.data(), 1000.0 * dx,
                shape.nx, shape.ny, shape.nz, out.ok ? out.field.run_line.c_str() : out.ok.error().context.c_str());
    return out;
}

// The run line's notes after its head ("solids: N parts, ..."), each
// "<part>: ...", in order.
[[nodiscard]] std::vector<std::string> run_line_notes(const std::string& line) {
    std::vector<std::string> out;
    std::size_t at = line.find("; ");
    while (at != std::string::npos) {
        const std::size_t next = line.find("; ", at + 2u);
        out.push_back(line.substr(at + 2u, next == std::string::npos ? std::string::npos : next - at - 2u));
        at = next;
    }
    return out;
}

// An arm's leak at an edge: "arm[<k>]: leaks at an edge (...)".
[[nodiscard]] bool arm_edge_leak(std::string_view note) {
    constexpr std::string_view arm = "arm[";
    constexpr std::string_view leak = "]: leaks at an edge (";
    if (!note.starts_with(arm)) return false;
    const std::size_t close = note.find(leak);
    if (close == std::string_view::npos || close == arm.size()) return false;
    for (std::size_t i = arm.size(); i < close; ++i) {
        if (note[i] < '0' || note[i] > '9') return false;
    }
    return true;
}

}  // namespace

// Ruling (a) in solids.cpp: an enclosed part is covered, not a leak.
constexpr bool kEnclosedPartsAreCovered = true;

// The five drones built to the solids rule (each part 8 mm or more across),
// each with its enclosed part and what the classifier said of it before (a):
// a pack or an ESC that closes no face was refused as a leak, or, crossed
// thin, was measured a sheet -- the drone's one sheet, every other part a wall.
// With (a) a drone builds, its run line's notes -- an arm's leak at an edge
// aside -- exactly `notes`, in order; or it is refused with exactly `exposed`.
// Pinned findings (Physics, 2026-10-27, reported to Core and the lead): the
// FPV's ESC, 36 mm across in z in a 40 mm stack, comes 2 mm from the stack's
// faces, inside 1 dx, and is exposed; the Phantom's and the Iris's bodies and
// the Iris's motor[3] leak at an edge.
struct Enclosed {
    std::string_view stem;
    std::string_view part;                  // the enclosed part
    bool refused;                           // before (a): refused as a leak, or else measured a sheet
    std::string_view exposed;               // with (a): the refusal in full; empty when the drone builds
    std::array<std::string_view, 4> notes;  // with (a): the run line's notes but an arm's edge leaks
};
constexpr std::array<Enclosed, 5> kDrones{{
    {"phantom3_class",
     "battery",
     true,
     "",
     {"body: leaks at an edge (3 runs, open area 0.667 dx^2, 0.04% of its largest face)", "esc: covered by 'body'",
      "battery: covered by 'body'"}},
    {"iris_plus_class",
     "battery",
     true,
     "",
     {"body: leaks at an edge (6 runs, open area 0.981 dx^2, 0.07% of its largest face)",
      "motor[3]: leaks at an edge (1 runs, open area 0.108 dx^2, 0.22% of its largest face)", "esc: covered by 'body'",
      "battery: covered by 'body'"}},
    {"fpv_5inch",
     "esc",
     true,
     "airflow solids: part 'esc' leaks: it closes no face of its own at dx = 4.0 mm and is exposed where it is not "
     "covered ('body', 'arm[0]', 'arm[1]', 'arm[2]' and 'arm[3]' close 817 of the 880 faces it would block; 63 stay "
     "open); declare min_feature or refine dx",
     {}},
    {"basic_quad_6inch", "esc", false, "", {"esc: covered by 'body'"}},
    {"basic_quad_competition", "esc", false, "", {"esc: covered by 'body'"}},
}};

// At 4 mm every rigid part of a drone that builds is a wall -- no sheet, one
// class per rigid part -- its enclosed parts covered by the body, and any leak
// at an edge (a note, within the classifier's own rule) an arm's, the meshed
// arms at 45 degrees leaving a few, but for the findings pinned above. Before
// (a) each named its enclosed part, which closes no face: refused by name, or
// the one sheet among walls.
TEST(LibraryDroneSolids, AtFourMillimetresEachDroneIsWallsOrNamesItsEnclosedPart) {
    for (const auto& [stem, part, refused, exposed, notes] : kDrones) {
        SCOPED_TRACE(stem);
        const DroneSolids b = build_drone_solids(stem, 0.004f);
        if (kEnclosedPartsAreCovered && !exposed.empty()) {
            EXPECT_FALSE(b.ok.has_value()) << b.field.run_line;
            if (b.ok.has_value()) continue;
            EXPECT_EQ(b.ok.error().context, exposed);
            continue;
        }
        // Nested, not `!kEnclosedPartsAreCovered && refused`: with the flag
        // true that is a constant condition, MSVC's C4127 (an error here).
        if (!kEnclosedPartsAreCovered) {
            if (refused) {
                EXPECT_FALSE(b.ok.has_value()) << b.field.run_line;
                if (b.ok.has_value()) continue;
                const std::string leaks = "part '" + std::string(part) + "' leaks: it closes no face";
                EXPECT_NE(b.ok.error().context.find(leaks), std::string::npos) << b.ok.error().context;
                continue;
            }
        }
        EXPECT_TRUE(b.ok.has_value()) << b.ok.error().context;
        if (!b.ok.has_value()) continue;
        EXPECT_EQ(b.field.classes.size(), b.rigid_parts);
        EXPECT_EQ(b.names.size(), b.field.classes.size());
        if (b.names.size() != b.field.classes.size()) continue;
        for (std::size_t i = 0; i < b.field.classes.size(); ++i) {
            const bool sheet = !kEnclosedPartsAreCovered && b.names[i] == part;
            EXPECT_EQ(b.field.classes[i].role, sheet ? SolidRole::sheet : SolidRole::wall) << b.names[i];
        }
        const std::size_t sheets = kEnclosedPartsAreCovered ? 0u : 1u;
        EXPECT_EQ(occurrences(b.field.run_line, ": sheet ("), sheets) << b.field.run_line;
        if (!kEnclosedPartsAreCovered) continue;
        std::vector<std::string> others;
        for (const std::string& note : run_line_notes(b.field.run_line)) {
            if (!arm_edge_leak(note)) {
                others.push_back(note);
            }
        }
        std::vector<std::string> expected;
        for (const std::string_view n : notes) {
            if (!n.empty()) {
                expected.emplace_back(n);
            }
        }
        EXPECT_EQ(others, expected) << b.field.run_line;
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
