// ---------------------------------------------------------------------------
// test_x500_case.cpp -- L86k (the lead, 2026-11-17): the website clips' X500, a flow model built from KAT's x500.airframe.json and
// kept with the clips' tool data (tools/flow_movie/data/x500/). L86p (the lead, 2026-11-18): the kit's propeller is the 1045 (10 x 4.5 in),
// R 0.127 m, the library's measured UIUC APC 10x5 blade at 10 in graded derived, no scaled estimate (tests/x500_model.hpp). Its mass, centre
// of mass and inertia are Core's parts table's (tools/flow_movie/data/x500_build/parts.yaml, core/x500-pose-table 1031a020; the lead, 2026-11-18):
// 13 parts, 1.6389 kg, not the Gazebo model's 2.0643 kg.
//
//   X500Model.TheFilesAreTheBuildersOutput       the model file and the prop part file are the builder's text, byte for byte; with
//                                                SPADE_X500_WRITE set the test writes them instead
//   X500Model.CarriesKatsNumbers                 mass 2.0643 kg, the four rotors at (+-0.174, +-0.174) m with PX4's spins, radius
//                                                0.127 m, the hubs 0.06 m above the centre of mass, the chain's propeller the 10-inch
//   X500Prop.NoRowIsEstimatedAndTheBladeIsTheApc10x5s   no row says ESTIMATED; diameter 0.254 m (graded for the kit), pitch derived with the
//                                                4.5 in against 5 in said, mass assumed; the blade's radius 0.127 m; c/R, twist and the
//                                                fitted polar the library's APC 10x5, as it is
//   X500Model.TheMassAndInertiaComeFromTheBuildTable   the compiled composite: 1.6389 kg, the centre of mass within 0.6 mm of the table's in x
//                                                and z and within 8 mm in y, the hubs at the table's pose (0.0614 m above its origin), the principal
//                                                diagonal within 3% of an independent sum of the table's parts, the products of inertia zero and
//                                                the design frame the principal frame (design_to_principal the identity)
//   X500Model.TheFilesComOffsetIsTheCentreOfMassInTheTablesFrame   the design frame IS the table's frame (the hubs 0.0614 m above its
//                                                origin, in the file too), so the model file's `com_offset` is the parts' centre of mass in the table's
//                                                frame, the one value a drawing of the table shifts by (Rendering: the table's origin sits at
//                                                -com_offset from the body)
//   X500Model.TheBatterySitsUnderTheFrameAsASolid   the 4S pack as a 0.133 x 0.0335 x 0.045 m box part under the frame stack
//   X500Balance.TheBalanceSpeedHoldsAQuarterOfTheWeight   the grid-free BEMT (tests/bemt_reference.hpp) bisected for W/4 = 4.0181 N:
//                                                thrust within 0.1% of the target; the speed against the chain's own static-thrust speed
//   AirflowX500.TheTwoAndAHalfMillimetreGridIsWithinTheCapWithItsSolids   266 x 237 x 266 at 2.5 mm; check_grid(kD1X500Grid, the case's
//                                                faces, with solids); the device runner's 4 GiB at 234.6 B a cell
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
        std::ofstream(x5::data_dir() / x5::kNotesFile, std::ios::binary | std::ios::trunc) << x5::notes_text();
        GTEST_SKIP() << "wrote " << model.string() << ", " << prop.string() << " and the notes";
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
    EXPECT_EQ(lf(read_all(x5::data_dir() / x5::kNotesFile)), x5::notes_text())
        << "x500.notes.yaml is stale or missing: regenerate with SPADE_X500_WRITE=1";
}

TEST(X500Model, CarriesKatsNumbers) {
    const auto file = sc::load_model_file(x5::data_dir() / x5::kModelFile);
    ASSERT_TRUE(file.has_value()) << file.error().context;
    const spade::vehicles::ModelType& m = file->model;
    ASSERT_EQ(m.rotors.size(), 4u);
    double mass = m.body.mass;
    EXPECT_NEAR(mass, 1.6389, 5.0e-4) << "the body's mass is the build table's 1.6389 kg (a rotor's own mass is in it), not KAT's 2.0643";
    const double expect_spin[4] = {+1.0, -1.0, +1.0, -1.0};  // front-right, front-left, rear-left, rear-right: PX4's X, as basic_quad lays it
    int quadrants = 0;
    for (std::size_t i = 0; i < 4; ++i) {
        const auto& r = m.rotors[i];
        EXPECT_NEAR(std::fabs(r.local_pos.x), x5::kArm, 2.0e-3) << "rotor " << i << ": 0.174 m from the centre of mass, to within its 0.9 mm offset";
        EXPECT_NEAR(std::fabs(r.local_pos.z), x5::kArm, 2.0e-3) << "rotor " << i;
        EXPECT_NEAR(r.local_pos.y, x5::kHubAboveCentre, 0.008) << "rotor " << i << ": the table's hub 0.0614 m above its origin, to within 8 mm";
        EXPECT_NEAR(r.radius, 0.127, 1.0e-4) << "rotor " << i << ": the 1045's 10 in";
        EXPECT_EQ(r.spin_dir, expect_spin[i]) << "rotor " << i;
        quadrants += (r.local_pos.x > 0 ? 1 : 0) * 2 + (r.local_pos.z > 0 ? 1 : 0);
    }
    EXPECT_EQ(quadrants, 0 + 1 + 2 + 3) << "the four rotors are in the four quadrants";
    ASSERT_TRUE(m.propulsion.has_value());
    EXPECT_NEAR(m.propulsion->diameter, 0.254, 1.0e-6) << "the chain's propeller is the 10-inch";
}

// The build table's parts (Core, parts.yaml at 1031a020), summed independently here: point masses, boxes for the frame stack, the pack, the
// ESCs and the misc carrier, thin tubes for the arms, cylinders for the motors. tests/p5_inertia.py-style numbers, kept as literals.
TEST(X500Model, TheMassAndInertiaComeFromTheBuildTable) {
    const auto prop = x5::prop_part();
    ASSERT_TRUE(prop.has_value()) << prop.error().context;
    const auto spec = x5::airframe(*prop);
    ASSERT_TRUE(spec.has_value()) << spec.error().context;
    const auto compiled = spade::vehicles::compile_airframe(*spec);
    ASSERT_TRUE(compiled.has_value()) << compiled.error().context;
    const auto& c = compiled->inertia;
    EXPECT_NEAR(c.mass, 1.6389f, 5.0e-4f) << "the 13 parts' sum";
    EXPECT_NEAR(c.center_of_mass.x, 0.00091f, 6.0e-4f) << "the table's centre of mass in x, from the parts' poses";
    EXPECT_NEAR(c.center_of_mass.z, -0.00159f, 6.0e-4f) << "and in z";
    EXPECT_NEAR(spec->rotors[0].hub.position.y, 0.0614, 1.0e-9) << "the hubs at the table's pose: the design frame is the table's frame";
    const double com_y_table = static_cast<double>(c.center_of_mass.y);
    EXPECT_NEAR(com_y_table, 0.0, 0.008) << "the parts' centre of mass is within 8 mm of the table's origin in y";
    const double hub_above = spec->rotors[0].hub.position.y - static_cast<double>(c.center_of_mass.y);
    const double want[3] = {0.014752, 0.022166, 0.015874};  // x roll, y yaw, z pitch about the centre of mass, design axes
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(c.inertia_design[i][i], want[i], 0.03 * want[i]) << "axis " << i;
    }
    // The products of inertia are dropped (the engine's model is principal-axes only; with I_x close to I_z they would turn the body frame).
    EXPECT_NEAR(c.inertia_design[0][1], 0.0, 2.0e-6);
    EXPECT_NEAR(c.inertia_design[0][2], 0.0, 2.0e-6);
    EXPECT_NEAR(c.inertia_design[1][2], 0.0, 2.0e-6);
    EXPECT_GT(std::fabs(compiled->model.design_to_principal.w), 0.99999f) << "the design frame is the principal frame: the rotors keep their poses";
    std::printf("[ x500    ] the parts' mass %.4f kg, centre of mass in the table's frame (%.4f, %.4f, %.4f) m, inertia diag (%.5f, %.5f, %.5f) kg m^2, "
                "hubs %.4f m above it\n",
                static_cast<double>(c.mass), static_cast<double>(c.center_of_mass.x), com_y_table, static_cast<double>(c.center_of_mass.z),
                c.inertia_design[0][0], c.inertia_design[1][1], c.inertia_design[2][2], hub_above);
}

// The design frame is the table's frame, so the compiled model's com_offset (the centre of mass from the design origin, design frame) is the
// centre of mass in the table's frame: one named value in the file, nothing copied by hand.
TEST(X500Model, TheFilesComOffsetIsTheCentreOfMassInTheTablesFrame) {
    const auto file = sc::load_model_file(x5::data_dir() / x5::kModelFile);
    ASSERT_TRUE(file.has_value()) << file.error().context;
    const spade::vehicles::ModelType& m = file->model;
    EXPECT_NEAR(m.com_offset.x, 0.00091f, 6.0e-4f);
    EXPECT_NEAR(m.com_offset.y, 0.0f, 0.008f) << "within 8 mm of the table's origin in y";
    EXPECT_NEAR(m.com_offset.z, -0.00159f, 6.0e-4f);
    ASSERT_EQ(m.rotors.size(), 4u);
    for (const auto& r : m.rotors) {
        EXPECT_NEAR(r.local_pos.y + m.com_offset.y, 0.0614f, 1.0e-4f) << "each hub 0.0614 m above the table's origin, in the file";
    }
    EXPECT_NEAR(m.rotors[0].local_pos.x + m.com_offset.x, 0.174f, 1.0e-4f) << "and at the table's x and z";
    EXPECT_NEAR(m.rotors[0].local_pos.z + m.com_offset.z, 0.174f, 1.0e-4f);
    std::printf("[ x500    ] model.com_offset (the parts' centre of mass in the table's frame): (%.5f, %.5f, %.5f) m\n",
                static_cast<double>(m.com_offset.x), static_cast<double>(m.com_offset.y), static_cast<double>(m.com_offset.z));
}

TEST(X500Model, TheBatterySitsUnderTheFrameAsASolid) {
    const auto file = sc::load_model_file(x5::data_dir() / x5::kModelFile);
    ASSERT_TRUE(file.has_value()) << file.error().context;
    const spade::vehicles::PartDesc* pack = nullptr;
    const spade::vehicles::PartDesc* body = nullptr;
    for (const auto& part : file->model.parts) {
        if (part.name == "battery") pack = &part;
        if (part.name == "body") body = &part;
    }
    ASSERT_TRUE(pack != nullptr) << "the pack is a part of the model";
    ASSERT_TRUE(body != nullptr);
    const auto box = spade::vehicles::part_bounds(*pack);
    ASSERT_TRUE(box.has_value()) << box.error().context;
    EXPECT_NEAR(box->max.x - box->min.x, 0.133f, 1.0e-3f);
    EXPECT_NEAR(box->max.y - box->min.y, 0.0335f, 1.0e-3f);
    EXPECT_NEAR(box->max.z - box->min.z, 0.045f, 1.0e-3f);
    const auto stack = spade::vehicles::part_bounds(*body);
    ASSERT_TRUE(stack.has_value()) << stack.error().context;
    const float pack_top = pack->mount_pos.y + box->max.y;
    const float stack_bottom = body->mount_pos.y + stack->min.y;
    EXPECT_LE(pack_top, stack_bottom + 1.0e-3f) << "under the frame stack, not inside it";
    EXPECT_GE(pack_top, stack_bottom - 2.0e-3f) << "touching it (the table's tray), no gap in the flow";
    EXPECT_NEAR(stack->max.y - stack->min.y, 0.0809f, 1.0e-3f) << "the stack from the tray's underside to the damper deck's top";
}

TEST(X500Prop, NoRowIsEstimatedAndTheBladeIsTheApc10x5s) {
    const auto text = read_all(x5::data_dir() / x5::kPropFile);
    ASSERT_FALSE(text.empty()) << "the prop part file is missing: regenerate with SPADE_X500_WRITE=1";
    const auto part = sc::prop_from_yaml(text);
    ASSERT_TRUE(part.has_value()) << part.error().context;
    EXPECT_NEAR(part->block.diameter, 0.254, 1.0e-9) << "the 1045's 10 in";
    EXPECT_FALSE(part->sources.empty());
    const sc::FieldSource* pitch = nullptr;
    const sc::FieldSource* diameter = nullptr;
    for (const sc::FieldSource& s : part->sources) {
        EXPECT_EQ(s.ref.note.find("ESTIMATED"), std::string::npos) << s.field << ": no scaled estimate any more: " << s.ref.note;
        if (s.field == "pitch") pitch = &s;
        if (s.field == "diameter") diameter = &s;
    }
    ASSERT_TRUE(pitch != nullptr && diameter != nullptr);
    EXPECT_EQ(pitch->ref.grade, sc::Grade::derived) << "5 in pitch (UIUC's 10x5) against the kit's 4.5 in";
    EXPECT_NE(pitch->ref.note.find("4.5"), std::string::npos) << pitch->ref.note;
    EXPECT_NE(diameter->ref.note.find("1045"), std::string::npos) << diameter->ref.note;
    const auto prop = sc::line_propeller(*part);
    ASSERT_TRUE(prop.has_value()) << prop.error().context;
    EXPECT_NEAR(prop->blade.radius, 0.127, 1.0e-6);
    const std::string base_text = read_all(fs::path(SPADE_ASSETS_DIR) / "library/parts/uiuc_apce_10x5.prop.yaml");
    const auto base = sc::prop_from_yaml(base_text);
    ASSERT_TRUE(base.has_value());
    ASSERT_TRUE(part->blade.has_value() && base->blade.has_value());
    EXPECT_EQ(part->blade->r_over_r, base->blade->r_over_r) << "r/R is the library's";
    EXPECT_EQ(part->blade->c_over_r, base->blade->c_over_r) << "c/R is the library's APC 10x5";
    EXPECT_EQ(part->blade->beta_deg, base->blade->beta_deg) << "the twist is the library's APC 10x5";
    EXPECT_EQ(part->blade->alpha_0_deg, base->blade->alpha_0_deg) << "the fitted polar is the library's";
    EXPECT_EQ(part->blade->cd_scale, base->blade->cd_scale) << "the fitted polar is the library's";
    EXPECT_EQ(part->block.ct_static, base->block.ct_static) << "the non-dimensional tables are the library's, as UIUC measured them";
    EXPECT_EQ(part->block.ct_table, base->block.ct_table);
}

TEST(X500Balance, TheBalanceSpeedHoldsAQuarterOfTheWeight) {
    const auto part = x5::prop_part();
    ASSERT_TRUE(part.has_value()) << part.error().context;
    const auto prop = sc::line_propeller(*part);
    ASSERT_TRUE(prop.has_value()) << prop.error().context;
    const float target = static_cast<float>(1.6389 * 9.80665 / 4.0);  // the build table's 1.6389 kg: W/4 = 4.0181 N
    const auto omega = x5::balance_omega(prop->blade, prop->polar, target);
    ASSERT_TRUE(omega.has_value()) << omega.error().context;
    spade::testing::BemtInput in;
    in.blade = &prop->blade;
    in.polar = &prop->polar;
    in.omega = *omega;
    in.tip_loss = true;
    const auto loads = spade::testing::bemt(in);
    ASSERT_TRUE(loads.has_value()) << loads.error().context;
    EXPECT_NEAR(loads->thrust, target, 0.001f * target) << "a quarter of the weight, 4.0181 N";
    // The chain's own static-thrust speed: ct_static rho n^2 D^4 = W/4.
    const double chain_n = std::sqrt(static_cast<double>(target) / (part->block.ct_static * 1.225 * std::pow(x5::kDiameter, 4)));
    const double chain_omega = 2.0 * 3.14159265358979 * chain_n;
    std::printf("[ x500    ] W/4 %.4f N: balance %.1f rad/s (%.0f rpm), BEMT torque %.4f N m; the chain's static-thrust speed %.1f rad/s "
                "(%.0f rpm); ratio %.3f; KAT's Gazebo speed would be 642 rad/s\n",
                static_cast<double>(target), static_cast<double>(*omega), static_cast<double>(*omega) * 60.0 / 6.283185307,
                static_cast<double>(loads->torque), chain_omega, chain_omega * 60.0 / 6.283185307,
                static_cast<double>(*omega) / chain_omega);
    EXPECT_GT(*omega, 520.0f);
    EXPECT_LT(*omega, 620.0f) << "a 10-inch prop holds 1.6389 kg near 5450 rpm";
    // The model's own hover speed: the chain's fit for a quarter of the model's weight, against the same BEMT balance.
    const auto file = sc::load_model_file(x5::data_dir() / x5::kModelFile);
    ASSERT_TRUE(file.has_value()) << file.error().context;
    EXPECT_NEAR(file->model.body.mass, 1.6389, 5.0e-4) << "the model file carries the build table's mass";
    const double model_n = std::sqrt(file->model.body.mass * 9.80665 / 4.0 / (part->block.ct_static * 1.225 * std::pow(x5::kDiameter, 4)));
    EXPECT_NEAR(2.0 * 3.14159265358979 * model_n, static_cast<double>(*omega), 0.02 * static_cast<double>(*omega))
        << "the chain's static-thrust speed for the model's own weight is within 2% of the BEMT balance";
}

TEST(AirflowX500, TheTwoAndAHalfMillimetreGridIsWithinTheCapWithItsSolids) {
    const spade::physics::airflow::GridShape grid = spade::testing::kD1X500Grid;
    EXPECT_EQ(grid.nx, 266u);
    EXPECT_EQ(grid.ny, 237u);
    EXPECT_EQ(grid.nz, 266u);
    EXPECT_EQ(grid.dx, 0.0025f);
    const auto made = spade::testing::still_air_case(x5::data_dir() / x5::kModelFile, "x500", grid);
    ASSERT_TRUE(made.has_value()) << made.error().context;
    const auto& bc = made->resolved.domain;
    const uint64_t fluid = spade::physics::airflow::fluid_bytes(grid, bc);
    const uint64_t solids = spade::physics::airflow::solid_bytes(grid, bc);
    const double cells = static_cast<double>(grid.nx) * grid.ny * grid.nz;
    std::printf("[ x500    ] %ux%ux%u at 2.5 mm: %.1f M cells; fluid_bytes %.3f GiB, solid_bytes %.3f GiB, together %.3f GiB against the cap "
                "%.3f GiB; the device runner's 234.6 B a cell: %.3f GiB of 4\n",
                grid.nx, grid.ny, grid.nz, cells / 1.0e6, static_cast<double>(fluid) / 1073741824.0,
                static_cast<double>(solids) / 1073741824.0, static_cast<double>(fluid + solids) / 1073741824.0,
                static_cast<double>(spade::physics::airflow::kGridByteCap) / 1073741824.0, cells * 234.6 / 1073741824.0);
    const auto ok = spade::physics::airflow::check_grid(grid, bc, true);
    EXPECT_TRUE(ok.has_value()) << (ok ? "" : ok.error().context);
    EXPECT_LE(cells * 234.6, 4.0 * 1073741824.0) << "the device runner's own 4 GiB (compute::vram_cap), measured at 234.6 B a cell";
}

// The line's kernel reaches R + 6 dx from a rotor's centre (the live run refuses a rotor whose reach leaves the grid, up or sideways):
// d1_x500_hold() leaves 1.5 R of open air above the hubs, and the box is wide enough for 0.174 + R + 6 dx from the axis.
TEST(AirflowX500, TheRotorsReachStaysInsideTheBoxWithRoomAbove) {
    const spade::physics::airflow::GridShape grid = spade::testing::kD1X500Grid;
    const auto file = sc::load_model_file(x5::data_dir() / x5::kModelFile);
    ASSERT_TRUE(file.has_value()) << file.error().context;
    const auto& r = file->model.rotors.front();
    EXPECT_NEAR(r.radius, 0.127, 1.0e-4);
    const float hub_above_origin = r.local_pos.y + file->model.com_offset.y;
    EXPECT_NEAR(spade::testing::d1_x500_hold().y, 0.075f - 13.0f * spade::testing::kD1X500Grid.dx, 1.0e-6f)
        << "the drone lowered 13 rows: the design origin is the table's, the hubs 0.0614 m above it";
    const float height = static_cast<float>(grid.ny) * grid.dx;
    const float top_margin = 0.5f * height - spade::testing::d1_x500_hold().y - hub_above_origin;
    const float reach = static_cast<float>(r.radius) + 6.0f * grid.dx;
    std::printf("[ x500    ] the rotors' centres are %.4f m under the top face (reach %.4f m, 1.5 R = %.4f m)\n",
                static_cast<double>(top_margin), static_cast<double>(reach), static_cast<double>(1.5f * r.radius));
    EXPECT_GE(top_margin, reach + 0.04f) << "the kernel's reach and 4 cells of room";
    EXPECT_GE(top_margin, 1.5f * static_cast<float>(r.radius) - 1.0e-4f) << "1.5 R of open air above the rotors";
    const float below = 0.5f * height + spade::testing::d1_x500_hold().y + hub_above_origin;
    EXPECT_GE(below, 0.40f - 0.002f) << "0.4 m of wake below the hubs";
    const float side = 0.5f * static_cast<float>(grid.nx) * grid.dx - (std::fabs(r.local_pos.x) + reach);
    std::printf("[ x500    ] sideways: %.4f m of room past the kernel's reach\n", static_cast<double>(side));
    EXPECT_GE(side, 0.0165f - 1.0e-4f) << "as much side room as the phantom's 2 mm box";
}

}  // namespace
