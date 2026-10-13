// A control program: the controls a run is flown with over time (I1; the CFD
// spec §5 "the control program"). It is held piecewise constant, a step at
// each point. One format, read and written only here (TD-9): a run folder
// holds it, LiveRun records it, and flow_movie and Kat's host fly it.
// showcase_program() is the one built-in program every front end flies.

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "analysis/controls.hpp"
#include "unique_temp.hpp"

namespace {

namespace an = spade::analysis;

[[nodiscard]] an::ControlPoint point(float t, bool armed, float throttle, float pitch = 0.0f, float roll = 0.0f,
                                     float yaw = 0.0f) {
    return an::ControlPoint{t, an::Controls{armed, throttle, pitch, roll, yaw}};
}

// Disarmed, then armed at half throttle, then a pitch and a roll.
[[nodiscard]] an::ControlProgram flight() {
    return an::ControlProgram{{point(0.0f, false, 0.0f), point(0.25f, true, 0.5f), point(1.0f, true, 0.5f, 12.5f),
                               point(1.5f, true, 0.55f, 0.0f, -10.0f, 30.0f)}};
}

[[nodiscard]] bool has_issue(const std::vector<an::CaseIssue>& issues, const std::string& field) {
    for (const an::CaseIssue& i : issues) {
        if (i.field == field) return true;
    }
    return false;
}

}  // namespace

// Piecewise constant: the last point at or before t.
TEST(AnalysisControls, HoldsEachPointUntilTheNext) {
    const an::ControlProgram p = flight();
    EXPECT_FALSE(an::controls_at(p, 0.0f).armed);
    EXPECT_FALSE(an::controls_at(p, 0.2499f).armed);
    EXPECT_TRUE(an::controls_at(p, 0.25f).armed);
    EXPECT_EQ(an::controls_at(p, 0.25f).throttle, 0.5f);
    EXPECT_EQ(an::controls_at(p, 0.99f).pitch_deg, 0.0f);
    EXPECT_EQ(an::controls_at(p, 1.0f).pitch_deg, 12.5f);
    // Past the last point, the last holds.
    EXPECT_EQ(an::controls_at(p, 100.0f).yaw_deg, 30.0f);
    EXPECT_EQ(an::controls_at(p, 100.0f).roll_deg, -10.0f);
}

TEST(AnalysisControls, ListsEveryProblemByField) {
    EXPECT_TRUE(an::program_issues(flight()).empty());
    EXPECT_TRUE(has_issue(an::program_issues(an::ControlProgram{}), "points"));
    an::ControlProgram p = flight();
    p.points[0].t_s = 0.1f;                                         // the first point must be at 0
    p.points[2].t_s = 0.25f;                                        // times must increase
    p.points[1].controls.throttle = 1.5f;                           // throttle in [0, 1]
    p.points[3].controls.yaw_deg = std::numeric_limits<float>::quiet_NaN();
    const std::vector<an::CaseIssue> issues = an::program_issues(p);
    EXPECT_TRUE(has_issue(issues, "points[0].t")) << issues.size();
    EXPECT_TRUE(has_issue(issues, "points[2].t"));
    EXPECT_TRUE(has_issue(issues, "points[1].throttle"));
    EXPECT_TRUE(has_issue(issues, "points[3].yaw_deg"));
}

// The file's text is canonical: written, read and written again, it is the
// same bytes.
TEST(AnalysisControls, RoundTripsByteForByte) {
    const spade::Result<std::string> text = an::program_to_yaml(flight());
    ASSERT_TRUE(text.has_value()) << text.error().context;
    EXPECT_EQ(text->rfind("version: 1\n", 0), 0u) << *text;
    const spade::Result<an::ControlProgram> back = an::program_from_yaml(*text);
    ASSERT_TRUE(back.has_value()) << back.error().context;
    ASSERT_EQ(back->points.size(), 4u);
    EXPECT_EQ(back->points[2].t_s, 1.0f);
    EXPECT_EQ(back->points[2].controls.pitch_deg, 12.5f);
    EXPECT_EQ(back->points[3].controls.yaw_deg, 30.0f);
    EXPECT_EQ(an::program_to_yaml(*back).value_or(""), *text);
}

TEST(AnalysisControls, RefusesWhatIsNotAProgram) {
    const std::string good = an::program_to_yaml(flight()).value_or("");
    std::string unknown = good;
    unknown.replace(unknown.find("throttle"), 8, "throttel");
    EXPECT_FALSE(an::program_from_yaml(unknown).has_value());
    std::string version = good;
    version.replace(0, 10, "version: 2");
    const spade::Result<an::ControlProgram> v2 = an::program_from_yaml(version);
    ASSERT_FALSE(v2.has_value());
    EXPECT_EQ(v2.error().code, spade::Code::schema_mismatch);
    // Parsed but not a valid program (its first point at 0.5 s): refused with
    // its first issue's field. The text is edited by hand, since the writer
    // would refuse to write it.
    std::string late_text = good;
    late_text.replace(late_text.find("{t: 0,"), 6, "{t: 0.5,");
    const spade::Result<an::ControlProgram> r = an::program_from_yaml(late_text);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("points[0].t"), std::string::npos) << r.error().context;
    // Writing refuses an invalid program too.
    an::ControlProgram late = flight();
    late.points[0].t_s = 0.5f;
    EXPECT_FALSE(an::program_to_yaml(late).has_value());
}

TEST(AnalysisControls, SavesAndLoads) {
    const spade::test::TempDir dir("controls");
    const std::filesystem::path file = dir.path() / "flight.controls.yaml";
    ASSERT_TRUE(an::save_program(flight(), file).has_value());
    const spade::Result<an::ControlProgram> back = an::load_program(file);
    ASSERT_TRUE(back.has_value()) << back.error().context;
    EXPECT_EQ(back->points.size(), 4u);
    const spade::Result<an::ControlProgram> missing = an::load_program(dir.path() / "none.controls.yaml");
    ASSERT_FALSE(missing.has_value());
    EXPECT_EQ(missing.error().code, spade::Code::io_error);
}

// The one built-in program: it starts disarmed, arms, and turns, so a film of
// it shows the spin-up and the drone in more than one attitude.
TEST(AnalysisControls, TheShowcaseProgramArmsAndTurns) {
    const an::ControlProgram& p = an::showcase_program();
    EXPECT_TRUE(an::program_issues(p).empty());
    EXPECT_FALSE(an::controls_at(p, 0.0f).armed);
    bool arms = false;
    bool turns = false;
    for (const an::ControlPoint& c : p.points) {
        arms = arms || (c.controls.armed && c.controls.throttle > 0.0f);
        turns = turns || c.controls.pitch_deg != 0.0f || c.controls.roll_deg != 0.0f || c.controls.yaw_deg != 0.0f;
    }
    EXPECT_TRUE(arms);
    EXPECT_TRUE(turns);
    EXPECT_GT(an::showcase_seconds(), p.points.back().t_s);
}
