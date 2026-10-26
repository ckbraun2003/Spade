// The editor's airflow run (INT-15, the plan
// docs/design/interface/plans/2026-10-22-editor-airflow-run-plan.md),
// display-free (sandbox/editor_airflow.hpp): T1's resolve line, what the
// inspector's Airflow section shows for a region before it runs.

#include <gtest/gtest.h>

#include <algorithm>
#include <format>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../sandbox/editor_airflow.hpp"
#include "../sandbox/editor_scene_edits.hpp"
#include "editor_test_support.hpp"

namespace ed = spade::sandbox::editor;
namespace support = spade::sandbox::editor::test;
using spade::Result;

namespace {

[[nodiscard]] spade::scene::SceneDesc with_region() {
    auto d = ed::make_scene_document(support::sample_scene(), {});
    const Result<void> added = ed::apply(d, ed::AddAirflowRegion{"hover_quad_0"});
    EXPECT_TRUE(added.has_value()) << (added ? "" : added.error().context);
    return d.desc();
}

}  // namespace

// The region's case resolves as LiveRun will run it, and the line says so as
// LiveRun's label does: "32³, dx 25 mm, R/dx 3".
TEST(EditorAirflow, ARegionsLineIsItsResolvedGrid) {
    const spade::scene::SceneDesc s = with_region();
    ASSERT_EQ(s.airflow.size(), 1u);
    const Result<spade::analysis::ResolvedCase> resolved = ed::air_case(s, s.airflow[0]);
    ASSERT_TRUE(resolved.has_value()) << resolved.error().context;
    EXPECT_EQ(resolved->base.nx, 32u);
    EXPECT_FLOAT_EQ(resolved->base.dx, s.airflow[0].cells.dx);
    const std::string line = ed::region_line(s, s.airflow[0]);
    EXPECT_TRUE(line.starts_with(std::format("32\xc2\xb3, dx {:g} mm, R/dx ", s.airflow[0].cells.dx * 1000.0f))) << line;
}

// A drone placed turned is refused before a step (the lead, 2026-10-26):
// LiveRun starts its drone level, and the air's box never turns.
TEST(EditorAirflow, ATurnedDroneIsRefusedNamingIt) {
    spade::scene::SceneDesc s = with_region();
    ASSERT_EQ(s.airflow.size(), 1u);
    for (auto& v : s.vehicles) v.start.orient = glm::angleAxis(0.5f, glm::vec3(0.0f, 1.0f, 0.0f));
    const Result<spade::analysis::ResolvedCase> resolved = ed::air_case(s, s.airflow[0]);
    ASSERT_FALSE(resolved.has_value());
    EXPECT_NE(resolved.error().context.find("'hover_quad_0'"), std::string::npos) << resolved.error().context;
    EXPECT_NE(resolved.error().context.find("Run air needs the drone unrotated"), std::string::npos)
        << resolved.error().context;
    EXPECT_EQ(ed::region_line(s, s.airflow[0]), resolved.error().context) << "the refusal, verbatim";
}

// The viewport draws each follow region as a wireframe box around its drone
// (T1): its 12 edges, centred where the drone is placed (LiveRun holds it at
// the box's centre), n dx on a side, in the region's line colour.
TEST(EditorAirflow, ARegionIsDrawnAsABoxAroundItsDrone) {
    spade::scene::SceneDesc s = with_region();
    ASSERT_EQ(s.airflow.size(), 1u);
    for (auto& v : s.vehicles) v.start.pos = glm::vec3(0.0f);
    spade::render::Camera cam;
    cam.position = glm::vec3(0.0f, 0.0f, 3.0f);
    const std::vector<ed::GizmoLine> lines = ed::region_box_lines(s, cam, 800, 600);
    ASSERT_EQ(lines.size(), 12u);
    const float half = 0.5f * static_cast<float>(s.airflow[0].cells.nx) * s.airflow[0].cells.dx;
    const std::optional<glm::vec2> centre = ed::project(cam, glm::vec3(0.0f), 800, 600);
    const std::optional<glm::vec2> near_corner = ed::project(cam, glm::vec3(half), 800, 600);
    ASSERT_TRUE(centre.has_value());
    ASSERT_TRUE(near_corner.has_value());
    float lo = 1e9f;
    float hi = -1e9f;
    for (const ed::GizmoLine& l : lines) {
        EXPECT_EQ(l.axis, ed::kRegionLine);
        lo = std::min({lo, l.a.x, l.b.x});
        hi = std::max({hi, l.a.x, l.b.x});
    }
    EXPECT_NEAR(hi, near_corner->x, 0.5f) << "the box's near face is its widest";
    EXPECT_NEAR(lo + hi, 2.0f * centre->x, 0.5f) << "centred on the drone";

    s.airflow.clear();
    EXPECT_TRUE(ed::region_box_lines(s, cam, 800, 600).empty());
}
