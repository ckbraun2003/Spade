// The drone CFD analysis's case (W0; INT-5, INT-6;
// docs/design/interface/plans/2026-10-12-drone-builder-cfd-design.md §4, §7):
// its file, its own checks, and resolving it against the model it pins.

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <string>

#include "analysis/case.hpp"
#include "core/rng.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "scene/scene_file.hpp"
#include "unique_temp.hpp"
#include "vehicles/model_identity.hpp"

namespace {

namespace an = spade::analysis;
using spade::Code;

// The hover scene's quadrotor: four rotors of 0.13 m.
[[nodiscard]] spade::vehicles::ModelType quad() {
    const auto s = spade::scene::load_scene_file(std::filesystem::path(SPADE_ASSETS_DIR) / "scenes" / "hover.scene.yaml");
    EXPECT_TRUE(s.has_value()) << (s ? "" : s.error().context);
    return s ? s->models.at(0) : spade::vehicles::ModelType{};
}

// A case with every section filled, pinned to `m`.
[[nodiscard]] an::AnalysisCase full_case(const spade::vehicles::ModelType& m) {
    an::AnalysisCase c;
    c.model = an::ModelRef{"quad.model.yaml", spade::vehicles::model_identity(m)};
    c.controls = an::Controls{.armed = true, .throttle = 0.55f, .pitch_deg = -5.0f, .roll_deg = 2.5f, .yaw_deg = 0.0f};
    c.flight = an::Flight{.kind = an::FlightKind::forward, .speed = 4.0f, .floor_height = 0.5f};
    c.grid = an::GridSetup{.box = 3.2f, .spacing = 0.025f, .nests = 2};
    for (std::size_t i = 0; i < m.rotors.size(); ++i) c.rotors.push_back(an::RotorSetup{an::RotorModel::line, "5x4"});
    an::BladeDefinition b;
    b.name = "5x4";
    b.blade_count = 2;
    b.radius = m.rotors.at(0).radius;
    b.hub_radius = 0.01f;
    b.collective_deg = 1.5f;
    b.stations = {an::BladeStation{0.2f, 0.012f, 22.0f, "clarky"}, an::BladeStation{0.6f, 0.016f, 12.5f, "clarky"},
                  an::BladeStation{1.0f, 0.008f, 6.0f, "tip"}};
    c.blades = {b};
    an::Airfoil a;
    a.id = "clarky";
    a.source = "XFOIL 6.99, Re 1e5, Ncrit 9";
    a.polars = {an::Polar{100000.0f, {-10.0f, 0.0f, 10.0f}, {-0.6f, 0.35f, 1.2f}, {0.03f, 0.012f, 0.05f}}};
    c.airfoils = {a};
    c.time = an::TimeSetup{.duration = 2.0f, .startup = 0.5f, .step = 0.0f};
    c.outputs = an::Outputs{.history_every = 0.01f, .snapshot_every = 0.25f};
    return c;
}

[[nodiscard]] bool names(const std::vector<an::CaseIssue>& issues, std::string_view field, std::string_view words) {
    for (const an::CaseIssue& i : issues) {
        if (i.field == field && i.message.find(words) != std::string::npos) return true;
    }
    return false;
}

[[nodiscard]] std::string listed(const std::vector<an::CaseIssue>& issues) {
    std::string out;
    for (const an::CaseIssue& i : issues) out += i.field + ": " + i.message + "\n";
    return out;
}

}  // namespace

// ---- the file ----

TEST(AnalysisCaseFile, RoundTripsByteForByte) {
    const an::AnalysisCase c = full_case(quad());
    const spade::Result<std::string> text = an::case_to_yaml(c);
    ASSERT_TRUE(text.has_value()) << text.error().context;
    const spade::Result<an::AnalysisCase> back = an::case_from_yaml(*text);
    ASSERT_TRUE(back.has_value()) << back.error().context;
    const spade::Result<std::string> again = an::case_to_yaml(*back);
    ASSERT_TRUE(again.has_value());
    EXPECT_EQ(*again, *text);
    EXPECT_EQ(back->model.identity, c.model.identity);
    EXPECT_EQ(back->blades.at(0).stations.at(1).twist_deg, 12.5f);
    EXPECT_EQ(back->airfoils.at(0).polars.at(0).alpha_deg.at(2), 10.0f);
    EXPECT_EQ(back->flight.kind, an::FlightKind::forward);
    EXPECT_EQ(back->rotors.at(3).model, an::RotorModel::line);
}

TEST(AnalysisCaseFile, SavesAndLoads) {
    const std::filesystem::path file = spade::test::unique_temp_path("quad.case.yaml");
    const an::AnalysisCase c = full_case(quad());
    ASSERT_TRUE(an::save_case(c, file).has_value());
    const spade::Result<an::AnalysisCase> back = an::load_case(file);
    std::filesystem::remove(file);
    ASSERT_TRUE(back.has_value()) << back.error().context;
    EXPECT_EQ(an::case_to_yaml(*back).value_or(""), an::case_to_yaml(c).value_or("-"));
}

TEST(AnalysisCaseFile, AnUnknownKeyIsRefusedWithItsLine) {
    std::string text = an::case_to_yaml(full_case(quad())).value_or("");
    const std::size_t at = text.find("  speed:");
    ASSERT_NE(at, std::string::npos);
    text.replace(at, 8, "  sped:");
    const spade::Result<an::AnalysisCase> c = an::case_from_yaml(text);
    ASSERT_FALSE(c.has_value());
    EXPECT_NE(c.error().context.find("sped"), std::string::npos) << c.error().context;
    EXPECT_NE(c.error().context.find("line"), std::string::npos) << c.error().context;
}

TEST(AnalysisCaseFile, ANewerVersionIsASchemaMismatch) {
    std::string text = an::case_to_yaml(full_case(quad())).value_or("");
    const std::size_t at = text.find("version: 1");
    ASSERT_NE(at, std::string::npos);
    text.replace(at, 10, "version: 2");
    const spade::Result<an::AnalysisCase> c = an::case_from_yaml(text);
    ASSERT_FALSE(c.has_value());
    EXPECT_EQ(c.error().code, Code::schema_mismatch);
}

// ---- the case's own fields ----

TEST(AnalysisCase, ItsOwnFieldsAreCheckedByName) {
    const spade::vehicles::ModelType m = quad();
    const auto refused = [&](auto change, std::string_view field) {
        an::AnalysisCase c = full_case(m);
        change(c);
        const spade::Result<void> r = an::validate_case(c);
        EXPECT_FALSE(r.has_value()) << field;
        if (!r) EXPECT_NE(r.error().context.find(field), std::string::npos) << r.error().context;
    };
    refused([](an::AnalysisCase& c) { c.controls.throttle = 1.5f; }, "controls.throttle");
    refused([](an::AnalysisCase& c) { c.grid.nests = 4; }, "grid.nests");
    refused([](an::AnalysisCase& c) { c.flight = an::Flight{an::FlightKind::hover, 2.0f, 0.0f}; }, "flight.speed");
    refused([](an::AnalysisCase& c) { c.flight = an::Flight{an::FlightKind::forward, 0.0f, 0.0f}; }, "flight.speed");
    refused([](an::AnalysisCase& c) { c.time.startup = c.time.duration; }, "time.startup");
    refused([](an::AnalysisCase& c) { c.fluid.density = 0.0f; }, "fluid.density");
    refused([](an::AnalysisCase& c) { c.blades.at(0).hub_radius = c.blades.at(0).radius; }, "blades[0].hub_radius");
    refused([](an::AnalysisCase& c) { c.blades.at(0).stations.at(1).r_over_radius = 0.1f; },
            "blades[0].stations[1].r_over_radius");
    refused([](an::AnalysisCase& c) { c.blades.at(0).stations.at(2).chord = 0.0f; }, "blades[0].stations[2].chord");
    refused([](an::AnalysisCase& c) { c.airfoils.at(0).polars.at(0).alpha_deg.at(1) = -20.0f; },
            "airfoils[0].polars[0].alpha_deg");
    refused([](an::AnalysisCase& c) { c.airfoils.at(0).polars.at(0).cd.pop_back(); }, "airfoils[0].polars[0].cd");
    refused([](an::AnalysisCase& c) { c.blades.push_back(c.blades.at(0)); }, "blades[1].name");
    EXPECT_TRUE(an::validate_case(full_case(m)).has_value());
}

// ---- resolving against the model ----

TEST(AnalysisCaseResolve, FillsTheDefaultsAndSaysSo) {
    const spade::vehicles::ModelType m = quad();
    an::AnalysisCase c = full_case(m);
    c.grid = an::GridSetup{};  // box and spacing from the drone
    for (an::RotorSetup& r : c.rotors) r.model = an::RotorModel::disc;
    const auto resolved = an::resolve(c, m);
    ASSERT_TRUE(resolved.has_value()) << listed(resolved.error());
    EXPECT_EQ(resolved->box_source, an::Source::default_value);
    EXPECT_EQ(resolved->spacing_source, an::Source::default_value);
    EXPECT_NEAR(resolved->spec.grid.box, an::kDefaultBoxReaches * an::reach_of(m), 1e-6f);
    EXPECT_NEAR(resolved->spec.grid.spacing, resolved->spec.grid.box / static_cast<float>(an::kDefaultCellsAcross), 1e-7f);
    EXPECT_EQ(resolved->base.nx, an::kDefaultCellsAcross);
    EXPECT_EQ(resolved->base_bytes, spade::physics::airflow::fluid_bytes(resolved->base));
    EXPECT_EQ(resolved->finest_spacing, resolved->spec.grid.spacing);
    // The hash names the run: the resolved case's own text.
    EXPECT_EQ(resolved->hash, spade::rng::fnv1a64(an::case_to_yaml(resolved->spec).value_or("")));
    const auto again = an::resolve(c, m);
    ASSERT_TRUE(again.has_value());
    EXPECT_EQ(again->hash, resolved->hash);
}

TEST(AnalysisCaseResolve, AGivenGridIsKeptAndNestsHalveTheCell) {
    const spade::vehicles::ModelType m = quad();
    const auto resolved = an::resolve(full_case(m), m);
    ASSERT_TRUE(resolved.has_value()) << listed(resolved.error());
    EXPECT_EQ(resolved->box_source, an::Source::given);
    EXPECT_EQ(resolved->spec.grid.spacing, 0.025f);
    EXPECT_EQ(resolved->finest_spacing, 0.025f / 4.0f);
    EXPECT_EQ(resolved->base.nx, 128u);  // 3.2 m / 2.5 cm
}

TEST(AnalysisCaseResolve, AChangedModelMustBeRepinned) {
    spade::vehicles::ModelType m = quad();
    const an::AnalysisCase c = full_case(m);
    m.body.mass += 0.1f;
    const auto resolved = an::resolve(c, m);
    ASSERT_FALSE(resolved.has_value());
    EXPECT_TRUE(names(resolved.error(), "model.identity", "re-pin")) << listed(resolved.error());
}

TEST(AnalysisCaseResolve, EveryModelRotorNeedsItsSetup) {
    const spade::vehicles::ModelType m = quad();
    an::AnalysisCase c = full_case(m);
    c.rotors.pop_back();
    const auto resolved = an::resolve(c, m);
    ASSERT_FALSE(resolved.has_value());
    EXPECT_TRUE(names(resolved.error(), "rotors", "4")) << listed(resolved.error());
}

TEST(AnalysisCaseResolve, TheLineNeedsTwentyCellsAcrossTheRadius) {
    const spade::vehicles::ModelType m = quad();
    an::AnalysisCase c = full_case(m);
    c.grid.nests = 0;  // a 2.5 cm cell: 0.13 m is 5.2 cells
    const auto refused = an::resolve(c, m);
    ASSERT_FALSE(refused.has_value());
    EXPECT_TRUE(names(refused.error(), "rotors[0]", "6.5 mm")) << listed(refused.error());  // R / 20
    c.grid.nests = 2;  // 6.25 mm: 20.8 cells
    EXPECT_TRUE(an::resolve(c, m).has_value());
}

TEST(AnalysisCaseResolve, BladeResolvedNeedsThirtyTwoCellsAcrossTheChord) {
    const spade::vehicles::ModelType m = quad();
    an::AnalysisCase c = full_case(m);
    c.rotors.at(1).model = an::RotorModel::blade_resolved;
    const auto refused = an::resolve(c, m);
    ASSERT_FALSE(refused.has_value());
    EXPECT_TRUE(names(refused.error(), "rotors[1]", "0.25 mm")) << listed(refused.error());  // 8 mm / 32
}

TEST(AnalysisCaseResolve, ABladeMustExistAndMatchItsRotor) {
    const spade::vehicles::ModelType m = quad();
    an::AnalysisCase c = full_case(m);
    c.rotors.at(2).blade = "";
    c.blades.at(0).radius = 0.2f;
    const auto refused = an::resolve(c, m);
    ASSERT_FALSE(refused.has_value());
    EXPECT_TRUE(names(refused.error(), "rotors[2].blade", "needs a blade")) << listed(refused.error());
    EXPECT_TRUE(names(refused.error(), "rotors[0].blade", "radius")) << listed(refused.error());
}

TEST(AnalysisCaseResolve, AnAirfoilWithoutPolarsIsEstimated) {
    const spade::vehicles::ModelType m = quad();
    const auto resolved = an::resolve(full_case(m), m);
    ASSERT_TRUE(resolved.has_value()) << listed(resolved.error());
    ASSERT_EQ(resolved->rotors.size(), m.rotors.size());
    // "tip" has no polars: the flat-plate default, labelled estimated.
    EXPECT_TRUE(resolved->rotors.at(0).estimated_polars);
    EXPECT_NEAR(resolved->rotors.at(0).cells_across_radius, m.rotors.at(0).radius / (0.025f / 4.0f), 1e-3f);
}

TEST(AnalysisCaseResolve, AWallFeatureNeedsTwoCells) {
    spade::vehicles::ModelType m = quad();
    spade::vehicles::PartDesc arm;
    arm.name = "arm";
    arm.shape.transforms.push_back(spade::SdfTransform{});
    spade::SdfNode n{};
    n.kind = static_cast<uint32_t>(spade::SdfPrim::sphere);
    n.op = static_cast<uint32_t>(spade::SdfOp::none);
    n.params = glm::vec4(0.01f, 0.0f, 0.0f, 0.0f);
    arm.shape.nodes = {n};
    arm.min_feature = 0.01f;  // 1 cm: 1.6 cells at 6.25 mm
    m.parts.push_back(arm);
    an::AnalysisCase c = full_case(m);
    const auto refused = an::resolve(c, m);
    ASSERT_FALSE(refused.has_value());
    EXPECT_TRUE(names(refused.error(), "parts[0]", "'arm'")) << listed(refused.error());
    EXPECT_TRUE(names(refused.error(), "parts[0]", "5 mm")) << listed(refused.error());
}

TEST(AnalysisCaseResolve, AGridPastTheCapIsRefusedBeforeRunning) {
    const spade::vehicles::ModelType m = quad();
    an::AnalysisCase c = full_case(m);
    for (an::RotorSetup& r : c.rotors) r.model = an::RotorModel::disc;
    c.grid = an::GridSetup{.box = 3.2f, .spacing = 0.002f, .nests = 0};  // 1600 cells across
    const auto refused = an::resolve(c, m);
    ASSERT_FALSE(refused.has_value());
    EXPECT_TRUE(names(refused.error(), "grid.spacing", "GiB")) << listed(refused.error());
}
