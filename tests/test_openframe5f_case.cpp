// ---------------------------------------------------------------------------
// test_openframe5f_case.cpp -- the website clips' flow model of the OpenFrame-5F, the 5-inch FPV kit Cameron chose (O138): tests/openframe5f_model.hpp
// builds it from Interface's build table (tools/flow_movie/data/openframe5f_build/parts.yaml) and the generator writes tools/flow_movie/data/openframe5f/.
// The checks are the X500's (tests/test_x500_case.cpp): the files are the builder's output, the rotors are the table's hubs, the composite is the table's
// mass, centre of mass and whole tensor (computed from the part store's meshes by tests/test_openframe5f_inertia.cpp), the proxy solids are where the
// table puts the parts, the propeller is graded a stand-in, and the hover speed is the grid-free balance's.
//
//   SPADE_OPENFRAME_WRITE=1   regenerate the data files instead of comparing them
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "openframe5f_model.hpp"
#include "scene/model_file.hpp"
#include "scene/part_file.hpp"
#include "scene/prop_line.hpp"
#include "vehicles/airframe_compile.hpp"

namespace {

namespace fs = std::filesystem;
namespace of = spade::testing::openframe5f;
namespace sc = spade::scene;

[[nodiscard]] bool write_asked() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv("SPADE_OPENFRAME_WRITE") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

[[nodiscard]] std::string read_all(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

[[nodiscard]] std::string lf(std::string s) {
    std::erase(s, '\r');
    return s;
}

struct Texts {
    std::string prop;
    std::string model;
};

[[nodiscard]] spade::Result<Texts> build_texts() {
    const auto prop = of::prop_part();
    if (!prop) return std::unexpected(prop.error());
    const auto spec = of::airframe(*prop);
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

// What the proxies leave for the remainder record: its mass, centre and the tensor about it (printed; the compile refuses a remainder that is not a body).
TEST(OpenFrame5fModel, TheRemainderIsWhatTheProxiesLeave) {
    const auto prop = of::prop_part();
    ASSERT_TRUE(prop.has_value()) << prop.error().context;
    auto spec = of::airframe(*prop);
    ASSERT_TRUE(spec.has_value()) << spec.error().context;
    ASSERT_EQ(spec->parts.back().geometry.name, "kit_remainder");
    const spade::vehicles::PartInertia rest = spec->parts.back();
    spec->parts.pop_back();
    const auto first = spade::vehicles::compile_airframe(*spec);
    ASSERT_TRUE(first.has_value()) << first.error().context;
    std::printf("[ openf5  ] proxies and blocks %.4f kg, com (%.5f, %.5f, %.5f); remainder %.4f kg at (%.5f, %.5f, %.5f), tensor diag (%.4e, %.4e, %.4e), "
                "xy %.3e xz %.3e yz %.3e
",
                static_cast<double>(first->inertia.mass), static_cast<double>(first->inertia.center_of_mass.x), static_cast<double>(first->inertia.center_of_mass.y),
                static_cast<double>(first->inertia.center_of_mass.z), rest.mass, rest.position.x, rest.position.y, rest.position.z, rest.tensor[0][0],
                rest.tensor[1][1], rest.tensor[2][2], rest.tensor[1][0], rest.tensor[2][0], rest.tensor[2][1]);
    const glm::dmat3 g = first->inertia.inertia_design;
    std::printf("[ openf5  ] proxies' tensor about their com: diag (%.4e, %.4e, %.4e) xy %.3e; the kit's diag (%.4e, %.4e, %.4e) xy %.3e
", g[0][0], g[1][1], g[2][2],
                g[1][0], of::kKitXX, of::kKitYY, of::kKitZZ, of::kKitXY);
}

TEST(OpenFrame5fModel, TheFilesAreTheBuildersOutput) {
    const auto texts = build_texts();
    ASSERT_TRUE(texts.has_value()) << texts.error().context;
    const fs::path model = of::data_dir() / of::kModelFile;
    const fs::path prop = of::data_dir() / of::kPropFile;
    if (write_asked()) {
        std::error_code ec;
        fs::create_directories(prop.parent_path(), ec);
        ASSERT_FALSE(ec) << ec.message();
        std::ofstream(model, std::ios::binary | std::ios::trunc) << texts->model;
        std::ofstream(prop, std::ios::binary | std::ios::trunc) << texts->prop;
        std::ofstream(of::data_dir() / of::kNotesFile, std::ios::binary | std::ios::trunc) << of::notes_text();
        GTEST_SKIP() << "wrote " << model.string() << ", " << prop.string() << " and the notes";
    }
    ASSERT_TRUE(fs::exists(model)) << model.string() << " is missing: regenerate with SPADE_OPENFRAME_WRITE=1";
    ASSERT_TRUE(fs::exists(prop)) << prop.string() << " is missing: regenerate with SPADE_OPENFRAME_WRITE=1";
    EXPECT_EQ(lf(read_all(model)), texts->model) << "openframe5f.model.yaml is stale: regenerate with SPADE_OPENFRAME_WRITE=1";
    EXPECT_EQ(lf(read_all(prop)), texts->prop) << "the prop part file is stale: regenerate with SPADE_OPENFRAME_WRITE=1";
    EXPECT_EQ(lf(read_all(of::data_dir() / of::kNotesFile)), of::notes_text()) << "the notes are stale or missing: regenerate with SPADE_OPENFRAME_WRITE=1";
}

// The table's hubs and spins (Interface, from Arm.step): a stretched X, +-0.069534 m along the kit's +X and +-0.089000 m across (+Z), the disc plane
// 0.0436 m above the origin (the motor's top 0.0396 m, 4 mm assumed), radius 0.0635 m; rotor order and spins as the library's fpv_5inch.
TEST(OpenFrame5fModel, CarriesTheTablesHubsSpinsAndMass) {
    const auto file = sc::load_model_file(of::data_dir() / of::kModelFile);
    ASSERT_TRUE(file.has_value()) << file.error().context;
    const spade::vehicles::ModelType& m = file->model;
    ASSERT_EQ(m.rotors.size(), 4u);
    EXPECT_NEAR(m.body.mass, 0.5665, 5.0e-5) << "the table's 14 entries' sum";
    const double expect_spin[4] = {+1.0, -1.0, +1.0, -1.0};
    const double sx[4] = {+1.0, +1.0, -1.0, -1.0};
    const double sz[4] = {+1.0, -1.0, -1.0, +1.0};
    for (std::size_t i = 0; i < 4; ++i) {
        const auto& r = m.rotors[i];
        const glm::vec3 design_hub(static_cast<float>(sx[i] * of::kArmX), static_cast<float>(of::kDiscHeight), static_cast<float>(sz[i] * of::kArmZ));
        const glm::vec3 want = m.design_to_principal * (design_hub - m.com_offset);
        EXPECT_NEAR(r.local_pos.x, want.x, 1.0e-5f) << "rotor " << i;
        EXPECT_NEAR(r.local_pos.y, want.y, 1.0e-5f) << "rotor " << i;
        EXPECT_NEAR(r.local_pos.z, want.z, 1.0e-5f) << "rotor " << i;
        EXPECT_NEAR(r.radius, 0.0635, 1.0e-5) << "rotor " << i << ": a 5 inch prop";
        EXPECT_EQ(r.spin_dir, expect_spin[i]) << "rotor " << i;
    }
    ASSERT_TRUE(m.propulsion.has_value());
    EXPECT_NEAR(m.propulsion->diameter, 0.127, 1.0e-6) << "the chain's propeller is the 5 inch stand-in";
}

// The compile's composite is the table's: mass, centre of mass and the whole tensor, products included (constants from the part store, checked by
// tests/test_openframe5f_inertia.cpp).
TEST(OpenFrame5fModel, TheMassAndInertiaComeFromTheBuildTable) {
    const auto prop = of::prop_part();
    ASSERT_TRUE(prop.has_value()) << prop.error().context;
    const auto spec = of::airframe(*prop);
    ASSERT_TRUE(spec.has_value()) << spec.error().context;
    const auto compiled = spade::vehicles::compile_airframe(*spec);
    ASSERT_TRUE(compiled.has_value()) << compiled.error().context;
    const auto& c = compiled->inertia;
    EXPECT_NEAR(c.mass, 0.5665f, 1.0e-6f);
    EXPECT_NEAR(c.center_of_mass.x, static_cast<float>(of::kKitCom.x), 2.0e-6f);
    EXPECT_NEAR(c.center_of_mass.y, static_cast<float>(of::kKitCom.y), 2.0e-6f);
    EXPECT_NEAR(c.center_of_mass.z, static_cast<float>(of::kKitCom.z), 2.0e-6f);
    const glm::dmat3 kit = of::kit_tensor();
    for (int i = 0; i < 3; ++i) {
        for (int k = 0; k < 3; ++k) EXPECT_NEAR(c.inertia_design[i][k], kit[i][k], 2.0e-8) << "tensor " << i << k;
    }
    EXPECT_NEAR(spec->rotors[0].hub.position.y, of::kDiscHeight, 1.0e-9) << "the disc 0.0436 m above the table's origin";
    const glm::quat q = compiled->model.design_to_principal;
    const double turn_deg = 2.0 * std::acos(std::min(1.0, static_cast<double>(std::fabs(q.w)))) * 180.0 / 3.14159265358979323846;
    EXPECT_NEAR(turn_deg, 2.88, 0.1) << "the principal axes' turn from the design axes, about z";
    std::printf("[ openf5  ] mass %.4f kg, centre of mass (%.5f, %.5f, %.5f) m in the table's frame, discs %.4f m above it, principal axes turn %.2f deg\n",
                static_cast<double>(c.mass), static_cast<double>(c.center_of_mass.x), static_cast<double>(c.center_of_mass.y),
                static_cast<double>(c.center_of_mass.z), of::kDiscHeight - static_cast<double>(c.center_of_mass.y), turn_deg);
}

TEST(OpenFrame5fModel, TheFilesComOffsetIsTheCentreOfMassInTheTablesFrame) {
    const auto file = sc::load_model_file(of::data_dir() / of::kModelFile);
    ASSERT_TRUE(file.has_value()) << file.error().context;
    const spade::vehicles::ModelType& m = file->model;
    EXPECT_NEAR(m.com_offset.x, static_cast<float>(of::kKitCom.x), 2.0e-6f);
    EXPECT_NEAR(m.com_offset.y, static_cast<float>(of::kKitCom.y), 2.0e-6f);
    EXPECT_NEAR(m.com_offset.z, static_cast<float>(of::kKitCom.z), 2.0e-6f);
    const glm::quat q_inverse = glm::inverse(m.design_to_principal);
    for (const auto& r : m.rotors) {
        const glm::vec3 design = q_inverse * r.local_pos + m.com_offset;
        EXPECT_NEAR(design.y, static_cast<float>(of::kDiscHeight), 1.0e-5f);
        EXPECT_NEAR(std::fabs(design.x), static_cast<float>(of::kArmX), 1.0e-5f);
        EXPECT_NEAR(std::fabs(design.z), static_cast<float>(of::kArmZ), 1.0e-5f);
    }
}

// The proxy solids stand where the table puts the parts: three plates and four arms from the kit's meshes' extents, the ESC board, the pack and the four
// motors; the remainder is a tensor record with no solid.
TEST(OpenFrame5fModel, TheProxySolidsStandWhereTheTablePutsTheParts) {
    const auto file = sc::load_model_file(of::data_dir() / of::kModelFile);
    ASSERT_TRUE(file.has_value()) << file.error().context;
    const spade::vehicles::ModelType& m = file->model;
    const glm::quat q_inverse = glm::inverse(m.design_to_principal);
    const auto find = [&](const std::string& name) -> const spade::vehicles::PartDesc* {
        for (const auto& part : m.parts) {
            if (part.name == name) return &part;
        }
        return nullptr;
    };
    for (const char* name : {"plate_bottom", "plate_middle", "plate_top", "arm[0]", "arm[1]", "arm[2]", "arm[3]", "battery", "esc", "motor[0]", "motor[1]",
                             "motor[2]", "motor[3]"}) {
        EXPECT_TRUE(find(name) != nullptr) << name << " is a part of the model";
    }
    EXPECT_TRUE(find("kit_remainder") == nullptr) << "the remainder is a tensor record, with no solid";
    const auto* pack = find("battery");
    ASSERT_TRUE(pack != nullptr);
    const auto pack_box = spade::vehicles::part_bounds(*pack);
    ASSERT_TRUE(pack_box.has_value()) << pack_box.error().context;
    EXPECT_NEAR(pack_box->max.x - pack_box->min.x, 0.078f, 1.0e-4f) << "the table's 78 x 38 x 39 mm pack";
    EXPECT_NEAR(pack_box->max.y - pack_box->min.y, 0.038f, 1.0e-4f);
    EXPECT_NEAR(pack_box->max.z - pack_box->min.z, 0.039f, 1.0e-4f);
    const glm::vec3 pack_at = q_inverse * pack->mount_pos + m.com_offset;
    EXPECT_NEAR(pack_at.x, -0.00385f, 1.0e-5f);
    EXPECT_NEAR(pack_at.y, 0.0535f, 1.0e-5f);
    EXPECT_NEAR(pack_at.z, 0.0f, 1.0e-5f);
    // Each motor's solid is the cylinder of the table's 2306 stand-in, its centre 0.0243 m up, and its top at the disc plane.
    for (int k = 0; k < 4; ++k) {
        const auto* motor = find("motor[" + std::to_string(k) + "]");
        ASSERT_TRUE(motor != nullptr);
        const glm::vec3 at = q_inverse * motor->mount_pos + m.com_offset;
        EXPECT_NEAR(at.y, 0.0243f, 1.0e-5f) << "motor " << k;
        EXPECT_NEAR(std::fabs(at.x), static_cast<float>(of::kArmX), 1.0e-5f);
        EXPECT_NEAR(std::fabs(at.z), static_cast<float>(of::kArmZ), 1.0e-5f);
    }
}

TEST(OpenFrame5fProp, TheBladeIsTheMit5x4AndItIsGradedAStandIn) {
    const auto text = read_all(of::data_dir() / of::kPropFile);
    ASSERT_FALSE(text.empty()) << "the prop part file is missing: regenerate with SPADE_OPENFRAME_WRITE=1";
    const auto part = sc::prop_from_yaml(text);
    ASSERT_TRUE(part.has_value()) << part.error().context;
    EXPECT_NEAR(part->block.diameter, 0.127, 1.0e-9) << "5 inch";
    EXPECT_NEAR(part->block.mass, 0.004, 1.0e-9);
    bool stand_in = false;
    for (const sc::FieldSource& s : part->sources) {
        EXPECT_EQ(s.ref.note.find("ESTIMATED"), std::string::npos) << s.field << ": " << s.ref.note;
        if (s.field == "ct_table" || s.field == "cq_table") {
            EXPECT_EQ(s.ref.grade, sc::Grade::derived) << s.field << ": another propeller's measured tables, used as a stand-in";
            stand_in = stand_in || s.ref.note.find("stand-in") != std::string::npos;
        }
    }
    EXPECT_TRUE(stand_in) << "the tables' notes say stand-in";
    const std::string base_text = read_all(fs::path(SPADE_ASSETS_DIR) / "library/parts/uiuc_mit_5x4.prop.yaml");
    const auto base = sc::prop_from_yaml(base_text);
    ASSERT_TRUE(base.has_value());
    ASSERT_TRUE(part->blade.has_value() && base->blade.has_value());
    EXPECT_EQ(part->blade->alpha_0_deg, base->blade->alpha_0_deg) << "the fitted polar is the library's";
    EXPECT_EQ(part->blade->cd_scale, base->blade->cd_scale);
    EXPECT_EQ(part->block.ct_table, base->block.ct_table);
    EXPECT_NE(of::kPropCaveat.find("6,057 rpm"), std::string_view::npos) << "the card's caveat line names the fit point";
}

// The hover speed from the table's mass: the grid-free balance for a quarter of the weight, against the chain's static-thrust speed.
TEST(OpenFrame5fBalance, TheBalanceSpeedHoldsAQuarterOfTheWeight) {
    const auto part = of::prop_part();
    ASSERT_TRUE(part.has_value()) << part.error().context;
    const auto prop = sc::line_propeller(*part);
    ASSERT_TRUE(prop.has_value()) << prop.error().context;
    const float target = static_cast<float>(0.5665 * 9.80665 / 4.0);
    const auto omega = of::balance_omega(prop->blade, prop->polar, target);
    ASSERT_TRUE(omega.has_value()) << omega.error().context;
    const double rpm = static_cast<double>(*omega) * 60.0 / 6.283185307;
    std::printf("[ openf5  ] W/4 %.4f N: balance %.1f rad/s (%.0f rpm)\n", static_cast<double>(target), static_cast<double>(*omega), rpm);
    EXPECT_GT(rpm, 8000.0);
    EXPECT_LT(rpm, 10500.0) << "a 5 inch tri-blade holds 0.5665 kg near 9,400 rpm";
}

}  // namespace
