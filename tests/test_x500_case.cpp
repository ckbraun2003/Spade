// ---------------------------------------------------------------------------
// test_x500_case.cpp -- L86k (the lead, 2026-11-17): the website clips' X500, a flow model built from KAT's x500.airframe.json and
// kept with the clips' tool data (tools/flow_movie/data/x500/), its 13-inch propeller an ESTIMATE (tests/x500_model.hpp).
//
//   X500Model.TheFilesAreTheBuildersOutput       the model file and the prop part file are the builder's text, byte for byte; with
//                                                SPADE_X500_WRITE set the test writes them instead
//   X500Model.CarriesKatsNumbers                 mass 2.0643 kg, the four rotors at (+-0.174, +-0.174) m with PX4's spins, radius
//                                                0.173 m, the hubs 0.06 m above the centre of mass, the chain's propeller the 13-inch
//   X500Prop.EveryRowIsGradedEstimated           each prop row grade assumed with "ESTIMATED" first; diameter 0.346 m; the blade's
//                                                radius 0.173 m; c/R, twist and the fitted polar the APC 10x5's
//   X500Balance.TheBalanceSpeedHoldsAQuarterOfTheWeight   the grid-free BEMT (tests/bemt_reference.hpp) bisected for W/4 = 5.0610 N:
//                                                thrust within 0.1% of the target; the speed against the chain's own static-thrust speed
//   AirflowX500.TheThreeMillimetreGridIsWithinTheCapWithItsSolids   check_grid(kD1X500Grid, the case's faces, with solids)
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "bemt_reference.hpp"
#include "d1_case.hpp"
#include "phantom_case.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "scene/model_file.hpp"
#include "scene/part_file.hpp"
#include "scene/prop_line.hpp"
#include "vehicles/airframe_compile.hpp"
#include "x500_model.hpp"

namespace {

namespace fs = std::filesystem;
namespace x5 = spade::testing::x500;
namespace sc = spade::scene;

[[nodiscard]] bool write_asked() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv("SPADE_X500_WRITE") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

[[nodiscard]] std::string read_all(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

struct Texts {
    std::string prop;
    std::string model;
};

[[nodiscard]] spade::Result<Texts> build_texts() {
    const auto prop = x5::prop_part();
    if (!prop) return std::unexpected(prop.error());
    const auto spec = x5::airframe(*prop);
    if (!spec) return std::unexpected(spec.error());
    const auto compiled = spade::vehicles::compile_airframe(*spec);
    if (!compiled) return std::unexpected(compiled.error());
    const auto chain = spade::vehicles::airframe_propulsion_chain(*spec);
    if (!chain) return std::unexpected(chain.error());
    sc::ModelFile file{compiled->model};
    file.model.propulsion = *chain;
    auto model = sc::model_to_yaml(file);
    if (!model) return std::unexpected(model.error());
    auto prop_text = sc::prop_to_yaml(*prop);
    if (!prop_text) return std::unexpected(prop_text.error());
    return Texts{std::move(*prop_text), std::move(*model)};
}

TEST(X500Model, TheFilesAreTheBuildersOutput) {
    const auto texts = build_texts();
    ASSERT_TRUE(texts.has_value()) << texts.error().context;
    const fs::path model = x5::data_dir() / x5::kModelFile;
    const fs::path prop = x5::data_dir() / x5::kPropFile;
    if (write_asked()) {
        std::error_code ec;
        fs::create_directories(prop.parent_path(), ec);
        ASSERT_FALSE(ec) << ec.message();
        std::ofstream(model, std::ios::binary | std::ios::trunc) << texts->model;
        std::ofstream(prop, std::ios::binary | std::ios::trunc) << texts->prop;
        GTEST_SKIP() << "wrote " << model.string() << " and " << prop.string();
    }
    ASSERT_TRUE(fs::exists(model)) << model.string() << " is missing: regenerate with SPADE_X500_WRITE=1";
    ASSERT_TRUE(fs::exists(prop)) << prop.string() << " is missing: regenerate with SPADE_X500_WRITE=1";
    // Line endings aside: a checkout with core.autocrlf gives the files CRLF.
    const auto lf = [](std::string s) {
        std::erase(s, '\r');
        return s;
    };
    EXPECT_EQ(lf(read_all(model)), texts->model) << "x500.model.yaml is stale: regenerate with SPADE_X500_WRITE=1";
    EXPECT_EQ(lf(read_all(prop)), texts->prop) << "the prop part file is stale: regenerate with SPADE_X500_WRITE=1";
}

TEST(X500Model, CarriesKatsNumbers) {
    const auto file = sc::load_model_file(x5::data_dir() / x5::kModelFile);
    ASSERT_TRUE(file.has_value()) << file.error().context;
    const spade::vehicles::ModelType& m = file->model;
    ASSERT_EQ(m.rotors.size(), 4u);
    double mass = m.body.mass;
    EXPECT_NEAR(mass, x5::kMass, 1.0e-4) << "the body's mass is KAT's 2.0643 kg (a rotor's own mass is in it)";
    const double expect_spin[4] = {+1.0, -1.0, +1.0, -1.0};  // front-right, front-left, rear-left, rear-right: PX4's X, as basic_quad lays it
    int quadrants = 0;
    for (std::size_t i = 0; i < 4; ++i) {
        const auto& r = m.rotors[i];
        EXPECT_NEAR(std::fabs(r.local_pos.x), x5::kArm, 1.0e-4) << "rotor " << i;
        EXPECT_NEAR(std::fabs(r.local_pos.z), x5::kArm, 1.0e-4) << "rotor " << i;
        EXPECT_NEAR(r.local_pos.y - m.com_offset.y * 0.0, x5::kHubAboveCentre, 0.01) << "rotor " << i << ": KAT's hub 0.06 m above the centre";
        EXPECT_NEAR(r.radius, x5::kDiameter / 2.0, 1.0e-4) << "rotor " << i;
        EXPECT_EQ(r.spin_dir, expect_spin[i]) << "rotor " << i;
        quadrants += (r.local_pos.x > 0 ? 1 : 0) * 2 + (r.local_pos.z > 0 ? 1 : 0);
    }
    EXPECT_EQ(quadrants, 0 + 1 + 2 + 3) << "the four rotors are in the four quadrants";
    ASSERT_TRUE(m.propulsion.has_value());
    EXPECT_NEAR(m.propulsion->diameter, x5::kDiameter, 1.0e-6) << "the chain's propeller is the 13-inch";
}

TEST(X500Prop, EveryRowIsGradedEstimated) {
    const auto text = read_all(x5::data_dir() / x5::kPropFile);
    ASSERT_FALSE(text.empty()) << "the prop part file is missing: regenerate with SPADE_X500_WRITE=1";
    const auto part = sc::prop_from_yaml(text);
    ASSERT_TRUE(part.has_value()) << part.error().context;
    EXPECT_NEAR(part->block.diameter, x5::kDiameter, 1.0e-9);
    EXPECT_FALSE(part->sources.empty());
    for (const sc::FieldSource& s : part->sources) {
        EXPECT_EQ(s.ref.grade, sc::Grade::assumed) << s.field;
        EXPECT_EQ(s.ref.note.rfind("ESTIMATED", 0), 0u) << s.field << ": " << s.ref.note;
    }
    const auto prop = sc::line_propeller(*part);
    ASSERT_TRUE(prop.has_value()) << prop.error().context;
    EXPECT_NEAR(prop->blade.radius, x5::kDiameter / 2.0, 1.0e-6);
    const std::string base_text = read_all(fs::path(SPADE_ASSETS_DIR) / "library/parts/uiuc_apce_10x5.prop.yaml");
    const auto base = sc::prop_from_yaml(base_text);
    ASSERT_TRUE(base.has_value());
    ASSERT_TRUE(part->blade.has_value() && base->blade.has_value());
    EXPECT_EQ(part->blade->c_over_r, base->blade->c_over_r) << "c/R is the APC 10x5's";
    EXPECT_EQ(part->blade->beta_deg, base->blade->beta_deg) << "the twist is the APC 10x5's";
    EXPECT_EQ(part->blade->alpha_0_deg, base->blade->alpha_0_deg) << "the fitted polar is the APC 10x5's";
}

TEST(X500Balance, TheBalanceSpeedHoldsAQuarterOfTheWeight) {
    const auto part = x5::prop_part();
    ASSERT_TRUE(part.has_value()) << part.error().context;
    const auto prop = sc::line_propeller(*part);
    ASSERT_TRUE(prop.has_value()) << prop.error().context;
    const float target = static_cast<float>(x5::kMass * 9.80665 / 4.0);
    const auto omega = x5::balance_omega(prop->blade, prop->polar, target);
    ASSERT_TRUE(omega.has_value()) << omega.error().context;
    spade::testing::BemtInput in;
    in.blade = &prop->blade;
    in.polar = &prop->polar;
    in.omega = *omega;
    in.tip_loss = true;
    const auto loads = spade::testing::bemt(in);
    ASSERT_TRUE(loads.has_value()) << loads.error().context;
    EXPECT_NEAR(loads->thrust, target, 0.001f * target) << "a quarter of the weight, 5.0610 N";
    // The chain's own static-thrust speed: ct_static rho n^2 D^4 = W/4.
    const double chain_n = std::sqrt(static_cast<double>(target) / (part->block.ct_static * 1.225 * std::pow(x5::kDiameter, 4)));
    const double chain_omega = 2.0 * 3.14159265358979 * chain_n;
    std::printf("[ x500    ] W/4 %.4f N: balance %.1f rad/s (%.0f rpm), BEMT torque %.4f N m; the chain's static-thrust speed %.1f rad/s "
                "(%.0f rpm); ratio %.3f; KAT's Gazebo speed would be 642 rad/s\n",
                static_cast<double>(target), static_cast<double>(*omega), static_cast<double>(*omega) * 60.0 / 6.283185307,
                static_cast<double>(loads->torque), chain_omega, chain_omega * 60.0 / 6.283185307,
                static_cast<double>(*omega) / chain_omega);
    EXPECT_GT(*omega, 250.0f);
    EXPECT_LT(*omega, 450.0f) << "a 13-inch prop holds this weight near 3300 rpm, not KAT's 6130";
}

TEST(AirflowX500, TheThreeMillimetreGridIsWithinTheCapWithItsSolids) {
    const spade::physics::airflow::GridShape grid = spade::testing::kD1X500Grid;
    const auto made = spade::testing::still_air_case(x5::data_dir() / x5::kModelFile, "x500", grid);
    ASSERT_TRUE(made.has_value()) << made.error().context;
    const auto& bc = made->resolved.domain;
    const uint64_t fluid = spade::physics::airflow::fluid_bytes(grid, bc);
    const uint64_t solids = spade::physics::airflow::solid_bytes(grid, bc);
    std::printf("[ x500    ] %ux%ux%u at 3 mm: %.1f M cells; fluid_bytes %.3f GiB, solid_bytes %.3f GiB, together %.3f GiB against the cap "
                "%.3f GiB\n",
                grid.nx, grid.ny, grid.nz, static_cast<double>(grid.nx) * grid.ny * grid.nz / 1.0e6,
                static_cast<double>(fluid) / 1073741824.0, static_cast<double>(solids) / 1073741824.0,
                static_cast<double>(fluid + solids) / 1073741824.0,
                static_cast<double>(spade::physics::airflow::kGridByteCap) / 1073741824.0);
    const auto ok = spade::physics::airflow::check_grid(grid, bc, true);
    EXPECT_TRUE(ok.has_value()) << (ok ? "" : ok.error().context);
}

}  // namespace
