// The drone CFD analysis's case (W0; INT-5, INT-6;
// docs/design/interface/plans/2026-10-12-drone-builder-cfd-design.md §4, §7):
// its file, its own checks, and resolving it against the model it pins.

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <exception>
#include <iterator>
#include <new>
#include <stdexcept>
#include <string>

#include "analysis/case.hpp"
#include "analysis/detail/caught.hpp"
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

// A wall part: a 1 cm sphere declaring `feature` as its smallest feature.
[[nodiscard]] spade::vehicles::PartDesc ball(std::string name, float feature) {
    spade::vehicles::PartDesc p;
    p.name = std::move(name);
    p.shape.transforms.push_back(spade::SdfTransform{});
    spade::SdfNode n{};
    n.kind = static_cast<uint32_t>(spade::SdfPrim::sphere);
    n.op = static_cast<uint32_t>(spade::SdfOp::none);
    n.params = glm::vec4(0.01f, 0.0f, 0.0f, 0.0f);
    p.shape.nodes = {n};
    p.min_feature = feature;
    return p;
}

// A case with every section filled, pinned to `m`.
[[nodiscard]] an::AnalysisCase full_case(const spade::vehicles::ModelType& m) {
    an::AnalysisCase c;
    c.model = an::ModelRef{"quad.model.yaml", spade::vehicles::model_identity(m), spade::vehicles::parts_identity(m)};
    c.controls = an::Controls{.armed = true, .throttle = 0.55f, .pitch_deg = -5.0f, .roll_deg = 2.5f, .yaw_deg = 0.0f};
    c.flight = an::Flight{.kind = an::FlightKind::forward, .speed = 4.0f, .heading_deg = 10.0f, .floor_height = 0.5f};
    c.grid = an::GridSetup{.box = 3.2f, .spacing = 0.025f, .nests = 2};
    for (std::size_t i = 0; i < m.rotors.size(); ++i) c.rotors.push_back(an::RotorSetup{an::RotorModel::line, "5x4"});
    an::BladeDefinition b;
    b.name = "5x4";
    b.blade_count = 2;
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
    c.time = an::TimeSetup{.duration = 2.0f, .startup = 0.5f, .step = std::nullopt};
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
    EXPECT_EQ(back->model.parts, c.model.parts);
    EXPECT_EQ(back->blades.at(0).stations.at(1).twist_deg, 12.5f);
    EXPECT_EQ(back->airfoils.at(0).polars.at(0).alpha_deg.at(2), 10.0f);
    EXPECT_EQ(back->flight.kind, an::FlightKind::forward);
    EXPECT_EQ(back->flight.heading_deg, 10.0f);
    EXPECT_EQ(back->rotors.at(3).model, an::RotorModel::line);
}

// What is absent is written as absent, and read back so: never a 0 that
// means "default" or "none".
TEST(AnalysisCaseFile, AnAbsentValueStaysAbsent) {
    an::AnalysisCase c = full_case(quad());
    c.grid = an::GridSetup{};
    c.flight = an::Flight{};
    c.outputs = an::Outputs{};
    const std::string text = an::case_to_yaml(c).value_or("");
    for (const char* key : {"box:", "spacing:", "floor_height:", "step:", "history_every:", "snapshot_every:"}) {
        EXPECT_EQ(text.find(key), std::string::npos) << key << " written though absent";
    }
    const spade::Result<an::AnalysisCase> back = an::case_from_yaml(text);
    ASSERT_TRUE(back.has_value()) << back.error().context;
    EXPECT_FALSE(back->grid.box.has_value());
    EXPECT_FALSE(back->grid.spacing.has_value());
    EXPECT_FALSE(back->flight.floor_height.has_value());
    EXPECT_FALSE(back->time.step.has_value());
    EXPECT_FALSE(back->outputs.history_every.has_value());
    EXPECT_FALSE(back->outputs.snapshot_every.has_value());
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

// A save replaces the file whole, through a temporary file beside it that
// doesn't outlive the save (Kat's ask): a reader sees the old case or the new.
// A reader that runs out of memory says so: the refusal is capacity_exceeded,
// "out of memory", never the io_error or parse failure a broken file gives
// (Spade Host's finding; the glTF importer's precedent). Every analysis
// reader's catch maps through caught_error(), so the rule is driven here
// directly: a real out-of-memory can't be induced reliably in a test.
TEST(AnalysisCaseFile, RunningOutOfMemoryIsNotAnIoFailure) {
    namespace detail = spade::analysis::detail;
    const spade::Error oom =
        detail::caught_error(std::make_exception_ptr(std::bad_alloc{}), spade::Code::io_error, "reading 'a.case.yaml'");
    EXPECT_EQ(oom.code, spade::Code::capacity_exceeded);
    EXPECT_NE(oom.context.find("out of memory"), std::string::npos) << oom.context;
    EXPECT_NE(oom.context.find("reading 'a.case.yaml'"), std::string::npos) << oom.context;

    const spade::Error io = detail::caught_error(std::make_exception_ptr(std::runtime_error("disk gone")),
                                                 spade::Code::io_error, "reading 'a.case.yaml'");
    EXPECT_EQ(io.code, spade::Code::io_error);
    EXPECT_NE(io.context.find("disk gone"), std::string::npos) << io.context;
    EXPECT_NE(io.context.find("reading 'a.case.yaml'"), std::string::npos) << io.context;

    const spade::Error other = detail::caught_error(std::make_exception_ptr(42), spade::Code::internal, "parsing");
    EXPECT_EQ(other.code, spade::Code::internal);
    EXPECT_NE(other.context.find("parsing"), std::string::npos) << other.context;
}

TEST(AnalysisCaseFile, ASaveReplacesTheFileWhole) {
    const spade::test::TempDir dir("case_save");
    const std::filesystem::path file = dir.path() / "quad.case.yaml";
    std::ofstream(file, std::ios::binary) << std::string(100000, '#');  // an older, longer file
    const an::AnalysisCase c = full_case(quad());
    ASSERT_TRUE(an::save_case(c, file).has_value());
    std::ifstream in(file, std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_EQ(text, an::case_to_yaml(c).value_or("-"));
    std::size_t files = 0;
    for (const auto& e : std::filesystem::directory_iterator(dir.path())) {
        (void)e;
        ++files;
    }
    EXPECT_EQ(files, 1u) << "the save left a file beside the case";
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
        if (!r) {
            EXPECT_NE(r.error().context.find(field), std::string::npos) << r.error().context;
        }
    };
    refused([](an::AnalysisCase& c) { c.controls.throttle = 1.5f; }, "controls.throttle");
    refused([](an::AnalysisCase& c) { c.grid.nests = 4; }, "grid.nests");
    refused(
        [](an::AnalysisCase& c) {
            c.flight = an::Flight{
                .kind = an::FlightKind::hover, .speed = 2.0f, .heading_deg = 0.0f, .floor_height = std::nullopt};
        },
        "flight.speed");
    refused(
        [](an::AnalysisCase& c) {
            c.flight = an::Flight{
                .kind = an::FlightKind::forward, .speed = 0.0f, .heading_deg = 0.0f, .floor_height = std::nullopt};
        },
        "flight.speed");
    refused([](an::AnalysisCase& c) { c.flight.floor_height = 0.0f; }, "flight.floor_height");
    refused([](an::AnalysisCase& c) { c.grid.spacing = 0.0f; }, "grid.spacing");
    refused([](an::AnalysisCase& c) { c.time.startup = c.time.duration; }, "time.startup");
    refused([](an::AnalysisCase& c) { c.fluid.density = 0.0f; }, "fluid.density");
    refused([](an::AnalysisCase& c) { c.blades.at(0).hub_radius = -0.01f; }, "blades[0].hub_radius");
    refused([](an::AnalysisCase& c) { c.blades.at(0).stations.at(1).r_over_radius = 0.1f; },
            "blades[0].stations[1].r_over_radius");
    refused([](an::AnalysisCase& c) { c.blades.at(0).stations.at(2).chord = 0.0f; }, "blades[0].stations[2].chord");
    refused([](an::AnalysisCase& c) { c.airfoils.at(0).polars.at(0).alpha_deg.at(1) = -20.0f; },
            "airfoils[0].polars[0].alpha_deg");
    refused([](an::AnalysisCase& c) { c.airfoils.at(0).polars.at(0).cd.pop_back(); }, "airfoils[0].polars[0].cd");
    refused([](an::AnalysisCase& c) { c.blades.push_back(c.blades.at(0)); }, "blades[1].name");
    EXPECT_TRUE(an::validate_case(full_case(m)).has_value());
}

// A host's form shows every problem at once; validate_case() is the first.
TEST(AnalysisCase, CaseIssuesListsEveryProblem) {
    an::AnalysisCase c = full_case(quad());
    c.controls.throttle = 1.5f;
    c.fluid.density = 0.0f;
    c.blades.at(0).stations.at(2).chord = 0.0f;
    const std::vector<an::CaseIssue> issues = an::case_issues(c);
    EXPECT_TRUE(names(issues, "controls.throttle", "[0, 1]")) << listed(issues);
    EXPECT_TRUE(names(issues, "fluid.density", "above 0")) << listed(issues);
    EXPECT_TRUE(names(issues, "blades[0].stations[2].chord", "above 0")) << listed(issues);
    const spade::Result<void> first = an::validate_case(c);
    ASSERT_FALSE(first.has_value());
    EXPECT_EQ(first.error().context, issues.front().field + ": " + issues.front().message);
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
    ASSERT_TRUE(resolved->spec.grid.box.has_value());
    ASSERT_TRUE(resolved->spec.grid.spacing.has_value());
    EXPECT_NEAR(*resolved->spec.grid.box, an::kDefaultBoxReaches * spade::vehicles::model_reach(m), 1e-6f);
    EXPECT_NEAR(*resolved->spec.grid.spacing, *resolved->spec.grid.box / static_cast<float>(an::kDefaultCellsAcross),
                1e-7f);
    EXPECT_EQ(resolved->base.nx, an::kDefaultCellsAcross);
    EXPECT_EQ(resolved->base_bytes, spade::physics::airflow::fluid_bytes(resolved->base, resolved->domain));
    EXPECT_EQ(resolved->finest_spacing, *resolved->spec.grid.spacing);
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
    m.parts.push_back(ball("guard", 0.02f));  // model_identity does not fold parts; parts_identity does
    const auto parts_changed = an::resolve(c, m);
    ASSERT_FALSE(parts_changed.has_value());
    EXPECT_TRUE(names(parts_changed.error(), "model.parts", "re-pin")) << listed(parts_changed.error());
    m = quad();
    m.body.mass += 0.1f;
    const auto body_changed = an::resolve(c, m);
    ASSERT_FALSE(body_changed.has_value());
    EXPECT_TRUE(names(body_changed.error(), "model.identity", "re-pin")) << listed(body_changed.error());
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

// A blade takes its rotor's radius: the case never holds a second one.
TEST(AnalysisCaseResolve, ABladeTakesItsRotorsRadius) {
    const spade::vehicles::ModelType m = quad();
    an::AnalysisCase c = full_case(m);
    c.rotors.at(2).blade = "";
    c.blades.at(0).hub_radius = m.rotors.at(0).radius;
    const auto refused = an::resolve(c, m);
    ASSERT_FALSE(refused.has_value());
    EXPECT_TRUE(names(refused.error(), "rotors[2].blade", "needs a blade")) << listed(refused.error());
    EXPECT_TRUE(names(refused.error(), "rotors[0].blade", "hub")) << listed(refused.error());

    const auto resolved = an::resolve(full_case(m), m);
    ASSERT_TRUE(resolved.has_value()) << listed(resolved.error());
    EXPECT_EQ(resolved->rotors.at(0).radius, m.rotors.at(0).radius);
    EXPECT_EQ(resolved->rotors.at(0).radius_source, an::Source::derived);
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
    m.parts.push_back(ball("arm", 0.01f));  // 1 cm: 1.6 cells at 6.25 mm
    const an::AnalysisCase c = full_case(m);
    const auto refused = an::resolve(c, m);
    ASSERT_FALSE(refused.has_value());
    EXPECT_TRUE(names(refused.error(), "parts[0]", "'arm'")) << listed(refused.error());
    EXPECT_TRUE(names(refused.error(), "parts[0]", "at most 5 mm")) << listed(refused.error());
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

// ---- vehicles::model_reach (Core's; the editor and the analysis share it) ----

TEST(ModelReach, IsTheFarthestRotorTipOrTheContactRadius) {
    spade::vehicles::ModelType m = quad();
    float tip = 0.0f;
    for (const spade::vehicles::RotorDesc& r : m.rotors) tip = std::max(tip, glm::length(r.local_pos) + r.radius);
    EXPECT_EQ(spade::vehicles::model_reach(m), std::max(tip, m.proxy_radius));
    m.proxy_radius = 10.0f;
    EXPECT_EQ(spade::vehicles::model_reach(m), 10.0f);
}

// The drone box's domain (Physics' plan A conditions): forward flight moves
// the air and the ground past the drone (the floor a rolling road); hover in a
// wind moves only the air; no face is periodic.
TEST(AnalysisCaseResolve, TheDomainIsTheDroneBox) {
    namespace af = spade::physics::airflow;
    const spade::vehicles::ModelType m = quad();
    const auto domain_of = [&](an::Flight f) {
        an::AnalysisCase c = full_case(m);
        for (an::RotorSetup& r : c.rotors) r.model = an::RotorModel::disc;
        c.flight = f;
        const auto r = an::resolve(c, m);
        EXPECT_TRUE(r.has_value()) << (r ? "" : listed(r.error()));
        return r ? r->domain : af::DomainBc{};
    };
    const float h = 10.0f * 3.14159265f / 180.0f;
    // Forward at 4 m/s, the air from 10 degrees left of the nose, over a floor.
    const af::DomainBc fwd = domain_of(an::Flight{.kind = an::FlightKind::forward, .speed = 4.0f,
                                                  .heading_deg = 10.0f, .floor_height = 0.5f});
    EXPECT_NEAR(fwd.ambient.x, -4.0f * std::cos(h), 1e-5f);
    EXPECT_NEAR(fwd.ambient.y, 0.0f, 1e-6f);
    EXPECT_NEAR(fwd.ambient.z, 4.0f * std::sin(h), 1e-5f);
    EXPECT_EQ(fwd.face[af::kYMinus], af::FaceBc::wall);
    EXPECT_EQ(fwd.wall_velocity[af::kYMinus], fwd.ambient);  // the rolling road
    for (const af::Side s : {af::kXMinus, af::kXPlus, af::kYPlus, af::kZMinus, af::kZPlus}) {
        EXPECT_EQ(fwd.face[s], af::FaceBc::open) << s;
    }
    // Hover in a 3 m/s wind from the left, over a floor that stays still.
    const af::DomainBc wind = domain_of(an::Flight{.kind = an::FlightKind::hover_in_wind, .speed = 3.0f,
                                                   .heading_deg = 90.0f, .floor_height = 0.5f});
    EXPECT_NEAR(wind.ambient.x, 0.0f, 1e-5f);
    EXPECT_NEAR(wind.ambient.z, 3.0f, 1e-5f);
    EXPECT_EQ(wind.face[af::kYMinus], af::FaceBc::wall);
    EXPECT_EQ(wind.wall_velocity[af::kYMinus], glm::vec3(0.0f));
    // Hover with no floor: still air, every face open.
    const af::DomainBc still = domain_of(an::Flight{.kind = an::FlightKind::hover, .speed = 0.0f,
                                                    .heading_deg = 0.0f, .floor_height = std::nullopt});
    EXPECT_EQ(still.ambient, glm::vec3(0.0f));
    EXPECT_EQ(still.face[af::kYMinus], af::FaceBc::open);
}
