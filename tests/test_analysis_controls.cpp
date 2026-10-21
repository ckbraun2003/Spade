// A control program: the controls a run is flown with over time (I1; the CFD
// spec §5 "the control program"). It is held piecewise constant, a step at
// each point. One format, read and written only here (TD-9): a run folder
// holds it, LiveRun records it, and flow_movie and Kat's host fly it.
// showcase_program() is the one built-in program every front end flies, around
// the drone's own trim (showcase_trim()).

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "analysis/controls.hpp"
#include "analysis/live_run.hpp"
#include "analysis_powered_quad.hpp"
#include "reference_airframe.hpp"
#include "unique_temp.hpp"
#include "vehicles/airframe_compile.hpp"
#include "vehicles/propulsion_steady.hpp"
#include "world/builder.hpp"

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

// The one built-in program, flown around one drone's trim: it starts
// disarmed, arms at hover, turns, climbs, and holds nothing else, so a film
// of it shows the spin-up and the drone in several attitudes and thrust
// states without driving any drone far past hover.
TEST(AnalysisControls, TheShowcaseProgramFliesAroundItsTrim) {
    const an::ShowcaseTrim trim{0.3f, 0.4f};
    const an::ControlProgram p = an::showcase_program(trim);
    EXPECT_TRUE(an::program_issues(p).empty());
    EXPECT_FALSE(an::controls_at(p, 0.0f).armed);
    bool arms = false;
    bool turns = false;
    bool climbs = false;
    for (const an::ControlPoint& c : p.points) {
        if (!c.controls.armed) {
            EXPECT_EQ(c.controls.throttle, 0.0f) << "t " << c.t_s;
            continue;
        }
        EXPECT_TRUE(c.controls.throttle == trim.hover || c.controls.throttle == trim.climb)
            << "t " << c.t_s << ": throttle " << c.controls.throttle;
        arms = arms || c.controls.throttle == trim.hover;
        climbs = climbs || c.controls.throttle == trim.climb;
        turns = turns || c.controls.pitch_deg != 0.0f || c.controls.roll_deg != 0.0f || c.controls.yaw_deg != 0.0f;
    }
    EXPECT_TRUE(arms);
    EXPECT_TRUE(climbs);
    EXPECT_TRUE(turns);
    EXPECT_GT(an::showcase_seconds(), p.points.back().t_s);
}

// ===========================================================================
// THE SHOWCASE TRIM (the lead, 2026-10-19). A fixed throttle reads differently
// on every drone: 0.5 commanded the reference quad about 5x its hover thrust.
// The trim is the drone's own: the throttle whose duty holds its weight
// (Physics' rule: steady_state_for_thrust() at m g / N per rotor, in the
// case's air at the propulsion's state of charge), and the one for 1.3 m g.
// ===========================================================================

namespace {

namespace vh = spade::vehicles;
using spade::analysis::test::powered_quad;
using spade::analysis::test::PoweredQuad;

// The weight in LiveRun's world (Environment{}'s gravity).
double weight_of(const vh::ModelType& m) {
    return static_cast<double>(m.body.mass) * static_cast<double>(-spade::Environment{}.gravity.y);
}

// The rotors' summed steady thrust at an armed throttle, in still air.
double thrust_at(const PoweredQuad& q, float throttle, double density) {
    const auto duty = vh::armed_duty(true, throttle, static_cast<double>(*q.power.idle));
    EXPECT_TRUE(duty.has_value());
    const vh::SteadyPoint p =
        vh::steady_state_at_duty(*q.model.propulsion, duty.value_or(0.0), density, 0.0,
                                 static_cast<double>(*q.power.state_of_charge));
    return p.thrust * static_cast<double>(q.model.rotors.size());
}

}  // namespace

TEST(AnalysisShowcaseTrim, TheTrimHoldsTheWeightAndClimbsAtOnePointThreeTimesIt) {
    const PoweredQuad q = powered_quad();
    const an::AnalysisCase c = spade::analysis::test::with_power(an::AnalysisCase{}, q);
    const spade::Result<an::ShowcaseTrim> trim = an::showcase_trim(c, q.model);
    ASSERT_TRUE(trim.has_value()) << trim.error().context;
    const double w = weight_of(q.model);
    EXPECT_NEAR(thrust_at(q, trim->hover, c.fluid.density) / w, 1.0, 1e-4);
    EXPECT_NEAR(thrust_at(q, trim->climb, c.fluid.density) / w, an::kShowcaseClimbThrust, 1e-4);
    EXPECT_GT(trim->hover, 0.0f);
    EXPECT_LT(trim->hover, trim->climb);
    EXPECT_LT(trim->climb, 1.0f);
}

// When the case's air and the pack match the compile's, the trim's hover is
// the airframe fit's hover_duty: one rule, two callers.
TEST(AnalysisShowcaseTrim, TheReferenceQuadHoversAtItsAirframeFitsDuty) {
    const vh::AirframeSpec spec = vh::fixtures::reference_quad();
    const auto compiled = vh::compile_airframe(spec);
    ASSERT_TRUE(compiled.has_value()) << compiled.error().context;
    const PoweredQuad q = powered_quad();
    an::AnalysisCase c = spade::analysis::test::with_power(an::AnalysisCase{}, q);
    c.fluid.density = static_cast<float>(spec.air_density);
    const spade::Result<an::ShowcaseTrim> trim = an::showcase_trim(c, q.model);
    ASSERT_TRUE(trim.has_value()) << trim.error().context;
    const auto duty = vh::armed_duty(true, trim->hover, static_cast<double>(*q.power.idle));
    ASSERT_TRUE(duty.has_value());
    EXPECT_NEAR(*duty, compiled->fit.hover_duty, 1e-6);
}

// Thinner air needs more duty to hold the same weight: the case's air, not
// the compile's.
TEST(AnalysisShowcaseTrim, TheTrimIsTheCasesAirNotTheCompiles) {
    const PoweredQuad q = powered_quad();
    const an::AnalysisCase sea = spade::analysis::test::with_power(an::AnalysisCase{}, q);
    an::AnalysisCase high = sea;
    high.fluid.density = 1.0f;
    const auto low_trim = an::showcase_trim(sea, q.model);
    const auto high_trim = an::showcase_trim(high, q.model);
    ASSERT_TRUE(low_trim.has_value() && high_trim.has_value());
    EXPECT_GT(high_trim->hover, low_trim->hover);
}

// What the lead found: the program, flown on the reference quad's trim, never
// commands more than the climb's thrust.
TEST(AnalysisShowcaseTrim, TheReferenceQuadsProgramStaysWithinTheClimbThrust) {
    const PoweredQuad q = powered_quad();
    const an::AnalysisCase c = spade::analysis::test::with_power(an::AnalysisCase{}, q);
    const auto trim = an::showcase_trim(c, q.model);
    ASSERT_TRUE(trim.has_value()) << trim.error().context;
    const double w = weight_of(q.model);
    for (const an::ControlPoint& p : an::showcase_program(*trim).points) {
        if (!p.controls.armed) continue;
        EXPECT_LE(thrust_at(q, p.controls.throttle, c.fluid.density) / w, an::kShowcaseClimbThrust + 1e-4)
            << "t " << p.t_s;
    }
}

TEST(AnalysisShowcaseTrim, AModelWithoutRotorsHasNoHoverAndAZeroTrim) {
    PoweredQuad q = powered_quad();
    q.model.rotors.clear();
    q.model.propulsion.reset();
    const auto trim = an::showcase_trim(an::AnalysisCase{}, q.model);
    ASSERT_TRUE(trim.has_value()) << trim.error().context;
    EXPECT_EQ(trim->hover, 0.0f);
    EXPECT_EQ(trim->climb, 0.0f);
}

TEST(AnalysisShowcaseTrim, RotorsWithoutPropulsionAreRefusedAsLiveRunRefusesThem) {
    PoweredQuad q = powered_quad();
    q.model.propulsion.reset();
    const auto trim = an::showcase_trim(an::AnalysisCase{}, q.model);
    ASSERT_FALSE(trim.has_value());
    EXPECT_EQ(trim.error().code, spade::Code::invalid_argument);
    EXPECT_NE(trim.error().context.find("needs their propulsion chain"), std::string::npos) << trim.error().context;
}

TEST(AnalysisShowcaseTrim, AChainThatCannotHoverTheModelIsRefusedByName) {
    PoweredQuad q = powered_quad();
    q.model.body.mass *= 100.0f;
    const auto trim = an::showcase_trim(spade::analysis::test::with_power(an::AnalysisCase{}, q), q.model);
    ASSERT_FALSE(trim.has_value());
    EXPECT_NE(trim.error().context.find("cannot hover"), std::string::npos) << trim.error().context;
}

// Hover in reach, 1.3 times it not: full thrust is 1.15 times the weight.
TEST(AnalysisShowcaseTrim, AClimbOutOfReachIsRefusedByName) {
    PoweredQuad q = powered_quad();
    const an::AnalysisCase c = spade::analysis::test::with_power(an::AnalysisCase{}, q);
    const double full =
        vh::steady_state_at_duty(*q.model.propulsion, 1.0, c.fluid.density, 0.0,
                                 static_cast<double>(*q.power.state_of_charge))
            .thrust *
        static_cast<double>(q.model.rotors.size());
    q.model.body.mass = static_cast<float>(full / 1.15 / static_cast<double>(-spade::Environment{}.gravity.y));
    const auto trim = an::showcase_trim(c, q.model);
    ASSERT_FALSE(trim.has_value());
    EXPECT_NE(trim.error().context.find("climb"), std::string::npos) << trim.error().context;
}

// Physics' bound: armed at zero throttle, an idle at or above the hover duty
// would climb.
TEST(AnalysisShowcaseTrim, AnIdleAtOrAboveTheHoverDutyIsRefusedInPhysicsWords) {
    PoweredQuad q = powered_quad();
    an::AnalysisCase c = spade::analysis::test::with_power(an::AnalysisCase{}, q);
    const double hover = vh::steady_state_for_thrust(*q.model.propulsion,
                                                     weight_of(q.model) / static_cast<double>(q.model.rotors.size()),
                                                     c.fluid.density, 0.0,
                                                     static_cast<double>(*q.power.state_of_charge))
                             .duty;
    c.power.idle = static_cast<float>(hover + 0.01);
    const auto trim = an::showcase_trim(c, q.model);
    ASSERT_FALSE(trim.has_value());
    EXPECT_NE(trim.error().context.find("at or above the hover duty"), std::string::npos) << trim.error().context;
}
