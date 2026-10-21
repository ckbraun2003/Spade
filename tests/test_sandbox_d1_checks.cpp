// The D1 hover preview's checks (docs/design/test-docs/2026-10-21-d1-hover-
// checks.md §1, §3): the body in the flow, no leak announced at 4 mm, the
// downwash with the body in the flow, the C_p legend's note, and a published
// volume with solids in it. Each check fails on a stub of the fault it exists
// to catch and passes on a stub of the right state.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "../sandbox/end_goal_checks.hpp"
#include "render/view_mode.hpp"

namespace eg = spade::sandbox::end_goal;

namespace {

[[nodiscard]] bool has(const std::string& s, const std::string& part) { return s.find(part) != std::string::npos; }

[[nodiscard]] bool judged_fail(const eg::Verdict& v) {
    return !v.ok && !has(v.detail, "absent:") && !has(v.detail, "mis-staged:") && !has(v.detail, "under-resolved:");
}

// The reference quad's line at 4 mm (tests/test_vehicles_airflow_solids.cpp,
// TheReferenceQuadAtHoverIsTenCleanWallsAt4Millimetres).
const std::string kAt4mm = "solids: 10 parts, 10 walls, 0 sheets";

// The same quad at 6 mm: two arms leak at an edge (the same test file pins
// parts[3] and parts[4]; the figures here are a stub's).
const std::string kAt6mm = "solids: 10 parts, 10 walls, 0 sheets; parts[3]: leaks at an edge (3 runs, open area "
                           "0.567 dx^2, 1.55% of its largest face); parts[4]: leaks at an edge (3 runs, open area "
                           "0.567 dx^2, 1.55% of its largest face)";

// A wall the safeguard measured as a sheet: it closes no face.
const std::string kNoClosedFace = "solids: 10 parts, 9 walls, 1 sheets; parts[1]: sheet (measured 5.1 mm across 12 "
                                  "thin crossings, no closed face; declared min_feature 12.0 mm)";

// A part declared thinner than 2 dx: a sheet, but not a leak.
const std::string kDeclaredSheet = "solids: 10 parts, 9 walls, 1 sheets; parts[1]: sheet (6.0 mm < 2 dx = 8.0 mm)";

[[nodiscard]] eg::SolidsLine line(std::string run_line, bool any = true, std::size_t expected = 10) {
    return {any, std::move(run_line), expected};
}

// render/view_mode.cpp's two notes for surface_cp, verbatim.
const std::string kNoteInFlow = "the fluid's pressure one cell off the wall, on the seen side: first order in the "
                                "cell size; where stagnation and suction are, not validated C_p magnitudes";
const std::string kNoteNoWalls = "the walls are not in this flow: the colour is the pressure of the flow passing "
                                 "through them, not a wall's";

[[nodiscard]] spade::render::ViewLegend cp_legend(std::string note) {
    spade::render::ViewLegend l;
    l.layer = spade::render::FlowView::surface_cp;
    l.quantity = "C_p";
    l.unit = "";
    l.range.min = -1.25f;
    l.range.max = 1.0f;
    l.range.from_header = true;
    l.note = std::move(note);
    return l;
}

[[nodiscard]] eg::LegendCheck shown_as_returned(std::vector<spade::render::ViewLegend> returned) {
    eg::LegendCheck c;
    c.layers = returned.size();
    c.returned = returned;
    c.shown = std::move(returned);
    c.region_running = false;
    return c;
}

}  // namespace

// ---------------------------------------------------------------------------
// D1-1. solids-in-flow
// ---------------------------------------------------------------------------

TEST(SandboxD1SolidsInFlow, PassesTheReferenceQuadsTenWallsAt4Millimetres) {
    const eg::Verdict v = eg::solids_in_flow(line(kAt4mm));
    EXPECT_TRUE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "10 parts, 10 walls, 0 sheets")) << v.detail;
}

TEST(SandboxD1SolidsInFlow, IsAbsentWhenThePartsWereCleared) {
    const eg::Verdict v = eg::solids_in_flow(line("solids: none", false));
    EXPECT_FALSE(v.ok);
    EXPECT_TRUE(has(v.detail, "absent:")) << v.detail;
}

TEST(SandboxD1SolidsInFlow, FailsRotorPartsLetInOrAPartClassedAsASheetOrAnUnreadLine) {
    const eg::Verdict rotors = eg::solids_in_flow(line("solids: 14 parts, 14 walls, 0 sheets"));
    EXPECT_TRUE(judged_fail(rotors)) << rotors.detail;
    EXPECT_TRUE(has(rotors.detail, "10 rigid parts")) << rotors.detail;
    const eg::Verdict sheet = eg::solids_in_flow(line(kDeclaredSheet));
    EXPECT_TRUE(judged_fail(sheet)) << sheet.detail;
    EXPECT_TRUE(has(sheet.detail, "parts[1]: sheet")) << sheet.detail;
    const eg::Verdict unread = eg::solids_in_flow(line("solids: 10 parts"));
    EXPECT_TRUE(judged_fail(unread)) << unread.detail;
}

TEST(SandboxD1SolidsInFlow, WithNoRigidPartsToExpectIsMisStaged) {
    const eg::Verdict v = eg::solids_in_flow(line(kAt4mm, true, 0));
    EXPECT_FALSE(v.ok);
    EXPECT_TRUE(has(v.detail, "mis-staged:")) << v.detail;
}

// ---------------------------------------------------------------------------
// D1-2. no-leak
// ---------------------------------------------------------------------------

TEST(SandboxD1NoLeak, PassesTheLineAt4Millimetres) {
    const eg::Verdict v = eg::no_leak(line(kAt4mm));
    EXPECT_TRUE(v.ok) << v.detail;
}

TEST(SandboxD1NoLeak, FailsAnEdgeLeakAt6MillimetresAndNamesThePart) {
    const eg::Verdict v = eg::no_leak(line(kAt6mm));
    EXPECT_TRUE(judged_fail(v)) << v.detail;
    EXPECT_TRUE(has(v.detail, "parts[3]: leaks at an edge")) << v.detail;
}

TEST(SandboxD1NoLeak, FailsAWallMeasuredWithNoClosedFace) {
    const eg::Verdict v = eg::no_leak(line(kNoClosedFace));
    EXPECT_TRUE(judged_fail(v)) << v.detail;
    EXPECT_TRUE(has(v.detail, "no closed face")) << v.detail;
}

TEST(SandboxD1NoLeak, ADeclaredSheetIsNotALeakButIsNotTheBody) {
    const eg::Verdict leak = eg::no_leak(line(kDeclaredSheet));
    EXPECT_TRUE(leak.ok) << leak.detail;
    EXPECT_TRUE(judged_fail(eg::solids_in_flow(line(kDeclaredSheet))));
}

TEST(SandboxD1NoLeak, IsAbsentWithNoSolids) {
    const eg::Verdict v = eg::no_leak(line("solids: none", false));
    EXPECT_FALSE(v.ok);
    EXPECT_TRUE(has(v.detail, "absent:")) << v.detail;
}

// ---------------------------------------------------------------------------
// D1-3. downwash-present, with the body in the flow
// ---------------------------------------------------------------------------

TEST(SandboxD1Downwash, IsMisStagedWithoutTheBodyInTheFlow) {
    const eg::Verdict in_flow = eg::solids_in_flow(line("solids: none", false));
    const eg::DownwashCase c{1.225f, 0.8f, 9.80665f};
    const eg::Verdict v = eg::downwash_with_body(in_flow, {}, c, eg::kDownwashPresent);
    EXPECT_FALSE(v.ok);
    EXPECT_TRUE(has(v.detail, "downwash-present: mis-staged: the body is not in the flow")) << v.detail;
}

TEST(SandboxD1Downwash, WithTheBodyInTheFlowItIsTheDownwashChecksOwnVerdict) {
    const eg::Verdict in_flow = eg::solids_in_flow(line(kAt4mm));
    ASSERT_TRUE(in_flow.ok) << in_flow.detail;
    const eg::DownwashCase c{1.225f, 0.8f, 9.80665f};
    const eg::Verdict with_body = eg::downwash_with_body(in_flow, {}, c, eg::kDownwashPresent);
    const eg::Verdict own = eg::downwash({}, c, eg::kDownwashPresent);
    EXPECT_EQ(with_body.ok, own.ok);
    EXPECT_EQ(with_body.detail, own.detail);
}

// ---------------------------------------------------------------------------
// D1-4. the C_p legend's note
// ---------------------------------------------------------------------------

TEST(SandboxD1CpNote, LegendShownFailsADroppedOrAlteredNote) {
    eg::LegendCheck dropped = shown_as_returned({cp_legend(kNoteInFlow)});
    dropped.shown[0].note.clear();
    eg::LegendCheck altered = shown_as_returned({cp_legend(kNoteInFlow)});
    altered.shown[0].note = "C_p on the walls";
    for (const eg::LegendCheck* c : {&dropped, &altered}) {
        const eg::Verdict v = eg::legend_shown(*c);
        EXPECT_TRUE(judged_fail(v)) << v.detail;
    }
    EXPECT_TRUE(eg::legend_shown(shown_as_returned({cp_legend(kNoteInFlow)})).ok);
}

TEST(SandboxD1CpNote, PassesTheSolidsNoteShownAndCarriedByTheCaption) {
    const std::vector<spade::render::ViewLegend> shown = {cp_legend(kNoteInFlow)};
    const eg::Verdict tour = eg::cp_note(shown);
    EXPECT_TRUE(tour.ok) << tour.detail;
    const std::string caption = "- Result: PASS\n\nC_p on the drone's walls (" + kNoteInFlow + ").";
    const eg::Verdict film = eg::cp_note(shown, caption);
    EXPECT_TRUE(film.ok) << film.detail;
}

TEST(SandboxD1CpNote, TheWallsNotInTheFlowIsMisStaged) {
    const std::vector<spade::render::ViewLegend> shown = {cp_legend(kNoteNoWalls)};
    const eg::Verdict v = eg::cp_note(shown);
    EXPECT_FALSE(v.ok);
    EXPECT_TRUE(has(v.detail, "mis-staged:")) << v.detail;
}

TEST(SandboxD1CpNote, FailsAnotherNoteOrACaptionWithoutIt) {
    const eg::Verdict other = eg::cp_note(std::vector<spade::render::ViewLegend>{cp_legend("pressure")});
    EXPECT_TRUE(judged_fail(other)) << other.detail;
    const eg::Verdict empty = eg::cp_note(std::vector<spade::render::ViewLegend>{cp_legend("")});
    EXPECT_TRUE(judged_fail(empty)) << empty.detail;
    const std::vector<spade::render::ViewLegend> shown = {cp_legend(kNoteInFlow)};
    const eg::Verdict captionless = eg::cp_note(shown, std::string("C_p on the drone's walls."));
    EXPECT_TRUE(judged_fail(captionless)) << captionless.detail;
}

TEST(SandboxD1CpNote, IsAbsentWithNoCpLegendShown) {
    spade::render::ViewLegend speed;
    speed.layer = spade::render::FlowView::streamlines;
    speed.quantity = "speed |u|";
    const eg::Verdict none = eg::cp_note(std::vector<spade::render::ViewLegend>{});
    const eg::Verdict other = eg::cp_note(std::vector<spade::render::ViewLegend>{speed});
    for (const eg::Verdict* v : {&none, &other}) {
        EXPECT_FALSE(v->ok);
        EXPECT_TRUE(has(v->detail, "absent:")) << v->detail;
    }
}

// ---------------------------------------------------------------------------
// D1-5. publishes, with solids. no_data counts samples, one per channel, so
// 500 interior cells are 4000 samples.
// ---------------------------------------------------------------------------

TEST(SandboxD1Publishes, PassesNoDataOnlyInTheSolidsInterior) {
    const std::vector<eg::Published> frames = {{11, true, 4000}, {12, true, 4000}};
    const eg::Verdict v = eg::publishes(frames, 500);
    EXPECT_TRUE(v.ok) << v.detail;
    EXPECT_TRUE(has(v.detail, "500")) << v.detail;
}

TEST(SandboxD1Publishes, FailsNoDataPastTheInteriorOrAMaskThatNeverReachedPublish) {
    const std::vector<eg::Published> past = {{11, true, 4000}, {12, true, 4001}};
    const std::vector<eg::Published> per_cell = {{11, true, 500}};
    const std::vector<eg::Published> unmasked = {{11, true, 0}, {12, true, 0}};
    for (const std::vector<eg::Published>* frames : {&past, &per_cell, &unmasked}) {
        const eg::Verdict v = eg::publishes(*frames, 500);
        EXPECT_TRUE(judged_fail(v)) << v.detail;
    }
    const std::vector<eg::Published> holed = {{11, true, 3}};
    EXPECT_TRUE(judged_fail(eg::publishes(holed)));
}
