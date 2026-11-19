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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "bemt_reference.hpp"
#include "d1_case.hpp"
#include "phantom_case.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/line.hpp"
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
    EXPECT_NEAR(mass, 1.6326, 5.0e-4) << "the body's mass is the build table's 1.6326 kg (a rotor's own mass is in it), not KAT's 2.0643";
    const double expect_spin[4] = {+1.0, -1.0, +1.0, -1.0};  // front-right, front-left, rear-left, rear-right: PX4's X, as basic_quad lays it
    int quadrants = 0;
    for (std::size_t i = 0; i < 4; ++i) {
        const auto& r = m.rotors[i];
        // The body frame is the design frame turned by design_to_principal and shifted by -com_offset (r_body = R (p_design - com_offset)): the
        // rotor is where the table's hub and disc height say, in the principal frame.
        const glm::vec3 design_hub(static_cast<float>((i == 0 || i == 1) ? x5::kArm : -x5::kArm), static_cast<float>(x5::kDiscHeight),
                                   static_cast<float>((i == 0 || i == 3) ? x5::kArm : -x5::kArm));
        const glm::vec3 want_pos = m.design_to_principal * (design_hub - m.com_offset);
        EXPECT_NEAR(r.local_pos.x, want_pos.x, 1.0e-5f) << "rotor " << i << ": the table's hub (+-0.17689, 0.0607, +-0.17689) in the principal frame";
        EXPECT_NEAR(r.local_pos.y, want_pos.y, 1.0e-5f) << "rotor " << i;
        EXPECT_NEAR(r.local_pos.z, want_pos.z, 1.0e-5f) << "rotor " << i;
        EXPECT_NEAR(r.radius, 0.127, 1.0e-4) << "rotor " << i << ": the 1045's 10 in";
        EXPECT_EQ(r.spin_dir, expect_spin[i]) << "rotor " << i;
        quadrants += (design_hub.x > 0 ? 1 : 0) * 2 + (design_hub.z > 0 ? 1 : 0);
    }
    EXPECT_EQ(quadrants, 0 + 1 + 2 + 3) << "the four rotors are in the four quadrants";
    ASSERT_TRUE(m.propulsion.has_value());
    EXPECT_NEAR(m.propulsion->diameter, 0.254, 1.0e-6) << "the chain's propeller is the 10-inch";
}

// The kit's composite (Core's parts.yaml at b1a5a7f0, the meshes' mass properties recorded in tests/x500_model.hpp, checked against the part store by
// tests/test_x500_kit_inertia.cpp): the compile's composite is the kit's mass, centre of mass and whole tensor, and the principal axes turn 5.4 degrees.
TEST(X500Model, TheMassAndInertiaComeFromTheBuildTable) {
    const auto prop = x5::prop_part();
    ASSERT_TRUE(prop.has_value()) << prop.error().context;
    const auto spec = x5::airframe(*prop);
    ASSERT_TRUE(spec.has_value()) << spec.error().context;
    const auto compiled = spade::vehicles::compile_airframe(*spec);
    ASSERT_TRUE(compiled.has_value()) << compiled.error().context;
    const auto& c = compiled->inertia;
    EXPECT_NEAR(c.mass, 1.6326f, 1.0e-5f) << "the 16 entries' sum";
    EXPECT_NEAR(c.center_of_mass.x, static_cast<float>(x5::kKitCom.x), 2.0e-6f) << "the kit's centre of mass from its meshes";
    EXPECT_NEAR(c.center_of_mass.y, static_cast<float>(x5::kKitCom.y), 2.0e-6f);
    EXPECT_NEAR(c.center_of_mass.z, static_cast<float>(x5::kKitCom.z), 2.0e-6f);
    // Core's own centre of mass (boxes and the frame's volume-weighted centre), 0.02, 0.38 and 0.32 mm away: the meshes confirm it.
    EXPECT_NEAR(c.center_of_mass.x, -0.00448f, 1.0e-3f);
    EXPECT_NEAR(c.center_of_mass.y, -0.02581f, 1.0e-3f);
    EXPECT_NEAR(c.center_of_mass.z, 0.00056f, 1.0e-3f);
    EXPECT_NEAR(spec->rotors[0].hub.position.y, x5::kDiscHeight, 1.0e-9) << "the discs at the table's height: the design frame is the table's frame";
    const double com_y_table = static_cast<double>(c.center_of_mass.y);
    const double hub_above = spec->rotors[0].hub.position.y - com_y_table;
    const glm::dmat3 kit = x5::kit_tensor();
    for (int i = 0; i < 3; ++i) {
        for (int k = 0; k < 3; ++k) EXPECT_NEAR(c.inertia_design[i][k], kit[i][k], 2.0e-6) << "tensor " << i << k << " is the kit's, products included";
    }
    // The products turn the principal axes 5.4 degrees about z; the file carries the turn (nothing dropped, nothing cancelled).
    const glm::quat q = compiled->model.design_to_principal;
    const double turn_deg = 2.0 * std::acos(std::min(1.0, static_cast<double>(std::fabs(q.w)))) * 180.0 / 3.14159265358979323846;
    EXPECT_NEAR(turn_deg, 5.4, 0.1) << "the principal axes' turn from the design axes";
    EXPECT_GT(std::fabs(q.z), 0.04f) << "about z";
    EXPECT_NEAR(q.x, 0.0f, 0.01f) << "the yz product (-5.5e-5 kg m^2) adds about 0.6 degrees about x";
    EXPECT_NEAR(q.y, 0.0f, 0.01f) << "and the xz product about 0.5 degrees about y, as the principal axes of the whole tensor";
    std::printf("[ x500    ] the parts' mass %.4f kg, centre of mass in the table's frame (%.4f, %.4f, %.4f) m, design-axes diag (%.5f, %.5f, %.5f) kg m^2, "
                "discs %.4f m above it; the principal axes turn %.2f degrees\n",
                static_cast<double>(c.mass), static_cast<double>(c.center_of_mass.x), com_y_table, static_cast<double>(c.center_of_mass.z),
                c.inertia_design[0][0], c.inertia_design[1][1], c.inertia_design[2][2], hub_above, turn_deg);
}

// The design frame is the table's frame, so the compiled model's com_offset (the centre of mass from the design origin, design frame) is the
// centre of mass in the table's frame: one named value in the file, nothing copied by hand.
TEST(X500Model, TheFilesComOffsetIsTheCentreOfMassInTheTablesFrame) {
    const auto file = sc::load_model_file(x5::data_dir() / x5::kModelFile);
    ASSERT_TRUE(file.has_value()) << file.error().context;
    const spade::vehicles::ModelType& m = file->model;
    EXPECT_NEAR(m.com_offset.x, static_cast<float>(x5::kKitCom.x), 2.0e-6f);
    EXPECT_NEAR(m.com_offset.y, static_cast<float>(x5::kKitCom.y), 2.0e-6f);
    EXPECT_NEAR(m.com_offset.z, static_cast<float>(x5::kKitCom.z), 2.0e-6f);
    ASSERT_EQ(m.rotors.size(), 4u);
    // Turned back into the design frame (p_design = R^-1 r_body + com_offset) each rotor is at the table's hub and disc height.
    const glm::quat q_inverse = glm::inverse(m.design_to_principal);
    for (const auto& r : m.rotors) {
        const glm::vec3 design = q_inverse * r.local_pos + m.com_offset;
        EXPECT_NEAR(design.y, static_cast<float>(x5::kDiscHeight), 1.0e-5f) << "each disc 0.0607 m above the table's origin, in the design frame";
        EXPECT_NEAR(std::fabs(design.x), static_cast<float>(x5::kArm), 1.0e-5f) << "and at the table's x and z";
        EXPECT_NEAR(std::fabs(design.z), static_cast<float>(x5::kArm), 1.0e-5f);
    }
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
    // The mounts are in the principal body frame: back to the design frame (p = R^-1 r + com_offset), where the parts' own axes are the table's.
    const glm::quat q_inverse = glm::inverse(file->model.design_to_principal);
    const float pack_y = (q_inverse * pack->mount_pos + file->model.com_offset).y;
    const float body_y = (q_inverse * body->mount_pos + file->model.com_offset).y;
    const float pack_top = pack_y + box->max.y;
    const float stack_bottom = body_y + stack->min.y;
    EXPECT_LE(pack_top, stack_bottom + 1.0e-3f) << "under the frame stack, not inside it";
    EXPECT_GE(pack_top, stack_bottom - 2.0e-3f) << "touching it (the table's tray), no gap in the flow";
    EXPECT_NEAR(stack->max.y - stack->min.y, 0.0805f, 1.0e-3f) << "the stack from the tray pad's underside to the top plate's top face";
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
    const float target = static_cast<float>(1.6326 * 9.80665 / 4.0);  // the build table's 1.6326 kg: W/4 = 4.0027 N
    const auto omega = x5::balance_omega(prop->blade, prop->polar, target);
    ASSERT_TRUE(omega.has_value()) << omega.error().context;
    spade::testing::BemtInput in;
    in.blade = &prop->blade;
    in.polar = &prop->polar;
    in.omega = *omega;
    in.tip_loss = true;
    const auto loads = spade::testing::bemt(in);
    ASSERT_TRUE(loads.has_value()) << loads.error().context;
    EXPECT_NEAR(loads->thrust, target, 0.001f * target) << "a quarter of the weight, 4.0027 N";
    // The chain's own static-thrust speed: ct_static rho n^2 D^4 = W/4.
    const double chain_n = std::sqrt(static_cast<double>(target) / (part->block.ct_static * 1.225 * std::pow(x5::kDiameter, 4)));
    const double chain_omega = 2.0 * 3.14159265358979 * chain_n;
    std::printf("[ x500    ] W/4 %.4f N: balance %.1f rad/s (%.0f rpm), BEMT torque %.4f N m; the chain's static-thrust speed %.1f rad/s "
                "(%.0f rpm); ratio %.3f; KAT's Gazebo speed would be 642 rad/s\n",
                static_cast<double>(target), static_cast<double>(*omega), static_cast<double>(*omega) * 60.0 / 6.283185307,
                static_cast<double>(loads->torque), chain_omega, chain_omega * 60.0 / 6.283185307,
                static_cast<double>(*omega) / chain_omega);
    EXPECT_GT(*omega, 520.0f);
    EXPECT_LT(*omega, 620.0f) << "a 10-inch prop holds 1.6326 kg near 5440 rpm";
    // The model's own hover speed: the chain's fit for a quarter of the model's weight, against the same BEMT balance.
    const auto file = sc::load_model_file(x5::data_dir() / x5::kModelFile);
    ASSERT_TRUE(file.has_value()) << file.error().context;
    EXPECT_NEAR(file->model.body.mass, 1.6326, 5.0e-4) << "the model file carries the build table's mass";
    const double model_n = std::sqrt(file->model.body.mass * 9.80665 / 4.0 / (part->block.ct_static * 1.225 * std::pow(x5::kDiameter, 4)));
    EXPECT_NEAR(2.0 * 3.14159265358979 * model_n, static_cast<double>(*omega), 0.02 * static_cast<double>(*omega))
        << "the chain's static-thrust speed for the model's own weight is within 2% of the BEMT balance";
}

TEST(AirflowX500, TheTwoAndAHalfMillimetreGridIsWithinTheCapWithItsSolids) {
    const spade::physics::airflow::GridShape grid = spade::testing::kD1X500Grid;
    EXPECT_EQ(grid.nx, 270u);
    EXPECT_EQ(grid.ny, 237u);
    EXPECT_EQ(grid.nz, 270u);
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
    const float hub_above_origin = static_cast<float>(x5::kDiscHeight);  // in the design frame, which the drone's origin is
    EXPECT_NEAR(spade::testing::d1_x500_hold().y, 0.075f - 13.0f * spade::testing::kD1X500Grid.dx, 1.0e-6f)
        << "the drone lowered 13 rows: the design origin is the table's, the discs 0.0607 m above it";
    const float height = static_cast<float>(grid.ny) * grid.dx;
    const float top_margin = 0.5f * height - spade::testing::d1_x500_hold().y - hub_above_origin;
    const float reach = static_cast<float>(r.radius) + 6.0f * grid.dx;
    std::printf("[ x500    ] the rotors' centres are %.4f m under the top face (reach %.4f m, 1.5 R = %.4f m)\n",
                static_cast<double>(top_margin), static_cast<double>(reach), static_cast<double>(1.5f * r.radius));
    EXPECT_GE(top_margin, reach + 0.04f) << "the kernel's reach and 4 cells of room";
    EXPECT_GE(top_margin, 1.5f * static_cast<float>(r.radius) - 1.0e-4f) << "1.5 R of open air above the rotors";
    const float below = 0.5f * height + spade::testing::d1_x500_hold().y + hub_above_origin;
    EXPECT_GE(below, 0.40f - 0.002f) << "0.4 m of wake below the hubs";
    const float side = 0.5f * static_cast<float>(grid.nx) * grid.dx - (static_cast<float>(x5::kArm) + reach);
    std::printf("[ x500    ] sideways: %.4f m of room past the kernel's reach\n", static_cast<double>(side));
    EXPECT_GE(side, 0.0165f - 1.0e-4f) << "as much side room as the phantom's 2 mm box";
}

// L105 (the lead, 2026-11-19), opt-in (SPADE_L105=1), CPU: why the lines' thrust at a fixed rotor speed falls with h. ONE of the kit's rotors (the 1045 at
// its balance speed) in a 0.40 x 0.55 x 0.40 m open box at 5 mm, the rotor 0.20 m under the top (the kernel reaches 0.157 m), its thrust at t = 0.01, 0.02 and 0.05 s for h 0.4, 0.2, 0.1 and 0.05 ms (the stills'
// 0.2 / 0.1 / 0.05 at half the cells, so the same cells a step at twice the h), in four modes:
//   as now      8 sub-azimuths, the field fed back (spread_line, then fluid_step)
//   frozen      8 sub-azimuths, the line reads a uniform downflow of the rotor's momentum-theory speed and the fluid is not stepped: no feedback
//   mid         ONE azimuth a step (n_sub 1, the step's middle), the field fed back
//   no-tip      as now with the tip/smearing correction off
// If frozen loses the h-dependence it is the feedback; if mid does, it is the sub-step sampling; if no-tip does, it is the correction.
[[nodiscard]] bool l105_asked() {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv("SPADE_L105") != nullptr;
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

[[nodiscard]] uint32_t l105_env(const char* name, uint32_t fallback) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const char* v = std::getenv(name);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    return v != nullptr ? static_cast<uint32_t>(std::strtoul(v, nullptr, 10)) : fallback;
}

// Optional: SPADE_L105_NV (the projection's V-cycles), SPADE_L105_MODE (0..3: only that mode), SPADE_L105_NH (the first NH of the four steps).
TEST(AirflowX500, TheLinesThrustAgainstTheStepInOneRotorOnTheCpu) {
    if (!l105_asked()) GTEST_SKIP() << "opt-in: SPADE_L105=1 (CPU, minutes)";
    namespace af = spade::physics::airflow;
    const auto part = x5::prop_part();
    ASSERT_TRUE(part.has_value()) << part.error().context;
    const auto prop = sc::line_propeller(*part);
    ASSERT_TRUE(prop.has_value()) << prop.error().context;
    const float target = static_cast<float>(x5::kMass * 9.80665 / 4.0);
    const auto omega = x5::balance_omega(prop->blade, prop->polar, target);
    ASSERT_TRUE(omega.has_value()) << omega.error().context;
    constexpr float kRho = 1.225f;
    const double area = 3.14159265358979 * 0.127 * 0.127;
    const float v_i = static_cast<float>(std::sqrt(static_cast<double>(target) / (2.0 * kRho * area)));  // m/s, momentum theory
    std::printf("[ L105    ] omega %.1f rad/s, momentum-theory induced speed %.3f m/s, W/4 %.4f N\n", static_cast<double>(*omega),
                static_cast<double>(v_i), static_cast<double>(target));
    struct Mode {
        const char* name;
        uint32_t n_sub;
        bool feedback;
        bool tip;
    };
    const Mode modes[] = {{"as now", 8, true, true}, {"frozen", 8, false, true}, {"mid", 1, true, true}, {"no-tip", 8, true, false}};
    const float hs[] = {0.0004f, 0.0002f, 0.0001f, 0.00005f};
    const double sample_t[] = {0.01, 0.02, 0.05};
    // SPADE_L105_DXMM (default 5): the cell, mm; the box in cells and every h scale with it, so the cells the blade moves a step are the same.
    const uint32_t dx_mm = l105_env("SPADE_L105_DXMM", 5u);
    const float dx = 0.001f * static_cast<float>(dx_mm);
    const float scale = static_cast<float>(dx_mm) / 5.0f;
    const uint32_t only_mode = l105_env("SPADE_L105_MODE", 99u);
    const uint32_t n_h = std::min<uint32_t>(l105_env("SPADE_L105_NH", 4u), 4u);
    for (uint32_t mi = 0; mi < 4u; ++mi) {
        if (only_mode != 99u && only_mode != mi) continue;
        const Mode& mode = modes[mi];
        std::printf("[ L105    ] %-7s (n_v %u)", mode.name, l105_env("SPADE_L105_NV", 2u));
        std::string rows;
        for (uint32_t hi = 0; hi < n_h; ++hi) {
            const float h = hs[hi] * scale;
            const af::GridShape shape{static_cast<uint32_t>(80.0f / scale), static_cast<uint32_t>(110.0f / scale), static_cast<uint32_t>(80.0f / scale), dx};
            af::DomainBc bc;
            auto grid = af::make_grid(shape, bc);
            ASSERT_TRUE(grid.has_value()) << grid.error().context;
            af::FluidConfig cfg;
            cfg.rho = kRho;
            cfg.h = h;
            cfg.n_v = l105_env("SPADE_L105_NV", cfg.n_v);
            auto scratch = af::make_fluid_scratch(*grid, bc, cfg);
            ASSERT_TRUE(scratch.has_value()) << scratch.error().context;
            if (!mode.feedback) std::fill(grid->v.begin(), grid->v.end(), -v_i);
            af::LineRotor rotor;
            rotor.centre = glm::vec3(0.5f * static_cast<float>(shape.nx) * dx, static_cast<float>(shape.ny) * dx - 0.20f,
                                     0.5f * static_cast<float>(shape.nz) * dx);
            rotor.axis = glm::vec3(0.0f, 1.0f, 0.0f);
            rotor.omega = *omega;
            rotor.blade = &prop->blade;
            rotor.polar = &prop->polar;
            rotor.tip_correction = mode.tip;
            const uint32_t total = l105_env("SPADE_L105_STEPS", static_cast<uint32_t>(std::lround(sample_t[2] / static_cast<double>(h))));
            double psi = 0.0;
            double first = 0.0;
            double early[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // the thrust at steps 1 to 6: what the blade reads of its own last steps' force
            double at[3] = {0.0, 0.0, 0.0};
            double inflow[3] = {0.0, 0.0, 0.0};   // m/s, the mean downward speed over the disc's area in the rotor's plane
            double inflow5[3] = {0.0, 0.0, 0.0};  // and 0.05 m under it
            // The mean downward air over the cells inside the disc's radius, on the v faces of row j.
            const auto disc_mean = [&](uint32_t j) {
                double sum = 0.0;
                uint32_t n = 0;
                for (uint32_t k = 0; k < shape.nz; ++k) {
                    for (uint32_t i = 0; i < shape.nx; ++i) {
                        const float x = (static_cast<float>(i) + 0.5f) * dx - rotor.centre.x;
                        const float z = (static_cast<float>(k) + 0.5f) * dx - rotor.centre.z;
                        if (x * x + z * z > 0.127f * 0.127f) continue;
                        sum -= static_cast<double>(grid->v[af::v_index(shape, i, j, k)]);
                        ++n;
                    }
                }
                return n != 0u ? sum / static_cast<double>(n) : 0.0;
            };
            const uint32_t row = static_cast<uint32_t>(std::lround(rotor.centre.y / dx));
            const auto t0 = std::chrono::steady_clock::now();
            for (uint32_t step = 1; step <= total; ++step) {
                rotor.psi = static_cast<float>(psi);
                const auto rep = af::spread_line(rotor, *grid, kRho, h, mode.n_sub, scratch->du, scratch->dv, scratch->dw);
                ASSERT_TRUE(rep.has_value()) << rep.error().context;
                if (step == 1) first = static_cast<double>(rep->thrust);
                if (step <= 6u) early[step - 1u] = static_cast<double>(rep->thrust);
                for (int k = 0; k < 3; ++k) {
                    if (step == static_cast<uint32_t>(std::lround(sample_t[k] / static_cast<double>(h)))) {
                        at[k] = static_cast<double>(rep->thrust);
                        inflow[k] = disc_mean(row);
                        inflow5[k] = disc_mean(row - static_cast<uint32_t>(std::lround(0.05f / dx)));
                    }
                }
                if (mode.feedback) {
                    const af::FluidHealth fh = af::fluid_step(*grid, bc, cfg, *scratch);
                    ASSERT_EQ(fh.fault, af::FluidFault::none) << af::describe(fh, grid->shape);
                }
                psi = std::fmod(psi + static_cast<double>(*omega) * static_cast<double>(h), 2.0 * 3.14159265358979);
            }
            const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            // Momentum theory for the thrust the line reports against the inflow the disc shows: T = 2 rho A v^2, so v_mt = sqrt(T / 2 rho A).
            const double v_mt = std::sqrt(at[2] / (2.0 * static_cast<double>(kRho) * area));
            char text[480];
            std::snprintf(text, sizeof text,
                          "\n[ L105    ]   h %.2f ms: step 1 %.3f N; 0.01 s %.3f, 0.02 s %.3f, 0.05 s %.3f N (%.0f s wall); at 0.05 s the disc's mean downflow "
                          "%.3f m/s in its plane, %.3f m/s 0.05 m under it, the thrust's momentum-theory speed %.3f m/s (disc/theory %.3f); at 0.02 s %.3f, %.3f m/s",
                          static_cast<double>(h) * 1000.0, first, at[0], at[1], at[2], wall, inflow[2], inflow5[2], v_mt, inflow[2] / v_mt, inflow[1], inflow5[1]);
            rows += text;
            std::snprintf(text, sizeof text, "\n[ L105    ]     steps 1 to 6: %.3f %.3f %.3f %.3f %.3f %.3f N (the blade moves %.2f cells a step)", early[0], early[1],
                          early[2], early[3], early[4], early[5],
                          static_cast<double>(*omega) * 0.127 * static_cast<double>(h) / static_cast<double>(dx));
            rows += text;
        }
        std::printf("%s\n", rows.c_str());
        std::fflush(stdout);
    }
}

}  // namespace
