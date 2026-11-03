// THE QUADROTOR'S LOOK (vehicles/basic_quad.hpp; B20-1, the lead's OK
// 2026-11-03): a QuadrotorParams airframe drawn as a basic quad. Each check
// poses the look as a drawing poses it (sandbox/vehicle_draw.hpp's body_at():
// the design frame through to_body_state(), then posed_parts()), so what it
// checks is what the drone box will draw: every hub on the airframe's, every
// rotor the airframe's radius and spin, and the parts as meshes.
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "sim/design_frame.hpp"
#include "state/layout.hpp"
#include "vehicles/basic_quad.hpp"
#include "vehicles/model_part.hpp"
#include "vehicles/part_pose.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/mesh.hpp"
#include "world/mesh_shapes.hpp"

namespace {

namespace vh = spade::vehicles;
namespace shapes = spade::shapes;
using vh::kQuadrotorRotorCount;
using MeshPtr = std::shared_ptr<const spade::TriangleMesh>;

// The drone box's stand (sandbox/drone_sim.hpp's drone_stand_params()): a plus
// of 0.18 m arms and R 0.12 m discs, whose neighbours are 14.6 mm clear.
[[nodiscard]] vh::QuadrotorParams stand() {
    vh::QuadrotorParams p;
    p.name = "stand";
    p.mass = 1.0f;
    p.inertia_diag = glm::vec3(0.018f, 0.032f, 0.024f);
    p.arm_length = 0.18f;
    for (vh::RotorParams& r : p.rotors) {
        r.thrust_coeff = 1.2e-5f;
        r.torque_coeff = 1.9e-7f;
    }
    return p;
}

// The caller's visual: the basic quad's parts, a square body (so it looks the
// same turned) and a look.
[[nodiscard]] vh::BasicQuadParams base() {
    vh::BasicQuadParams b;
    b.name = "stand_look";
    b.body_size = glm::dvec3(0.09, 0.035, 0.09);
    b.look = vh::BasicQuadLook{.body_edge_radius = 0.007, .bell_chamfer = 0.00145, .segments = 4};
    return b;
}

// A blade of two stations, as a prop part file gives one; its diameter is the
// look's to set.
[[nodiscard]] shapes::PropMeshSpec blade() {
    shapes::PropMeshSpec m;
    m.diameter = 0.1524;
    m.blades = 3;
    m.r_over_r = {0.2, 1.0};
    m.c_over_r = {0.15, 0.08};
    m.beta_deg = {20.0, 10.0};
    m.hub_cutout = 0.15;
    m.hub_gap = 0.0005;
    m.hub_height = 0.008;
    m.hub_segments = 16;
    m.section = shapes::PropSection{
        .camber = 0.04, .camber_at = 0.4, .thickness_root = 0.12, .thickness_tip = 0.08, .chord_points = 8};
    return m;
}

// The look's parts for the airframe's body upright at rest at the origin (so
// world is the airframe's body frame), every rotor at `omega`, psi 0.
[[nodiscard]] spade::Result<vh::PosedParts> posed(const vh::QuadrotorLook& look, float omega) {
    const spade::FrameState design{look.offset, look.turn, glm::vec3(0.0f), glm::vec3(0.0f)};
    const spade::FrameState b =
        spade::to_body_state(design, glm::normalize(look.model.design_to_principal), look.model.com_offset);
    spade::BodyState body{};
    body.pos = b.pos;
    body.orient = b.orient;
    const std::vector<float> w(look.model.rotors.size(), omega);
    const std::vector<float> psi(look.model.rotors.size(), 0.0f);
    return vh::posed_parts(look.model, body, w, psi, vh::PartFilter::all);
}

// The airframe rotor drawn by the model's rotor `j`, or kQuadrotorRotorCount.
[[nodiscard]] std::size_t airframe_rotor(const vh::QuadrotorLook& look, uint32_t j) {
    for (std::size_t i = 0; i < kQuadrotorRotorCount; ++i) {
        if (look.rotor[i] == j) return i;
    }
    return kQuadrotorRotorCount;
}

TEST(QuadrotorLook, DrawsEveryHubWhereTheAirframeHasIt) {
    for (const float height : {0.0f, 0.03f}) {
        vh::QuadrotorParams airframe = stand();
        airframe.rotor_height = height;
        const spade::Result<vh::QuadrotorLook> look = vh::quadrotor_look(airframe, base(), blade());
        ASSERT_TRUE(look.has_value()) << look.error().context;
        const spade::Result<vh::PosedParts> parts = posed(*look, 0.0f);
        ASSERT_TRUE(parts.has_value()) << parts.error().context;
        std::array<int, kQuadrotorRotorCount> props{};
        for (const vh::PosedPart& pp : parts->parts) {
            const vh::PartDesc& part = look->model.parts[pp.part];
            if (part.attach != vh::PartAttach::rotor) continue;
            const std::size_t i = airframe_rotor(*look, part.rotor);
            ASSERT_LT(i, kQuadrotorRotorCount) << part.name << " is on no airframe rotor";
            ++props[i];
            const glm::vec3 hub = vh::quadrotor_arm_offset(airframe, i);
            for (int a = 0; a < 3; ++a) {
                EXPECT_NEAR(pp.hub[a], hub[a], 1e-6f) << "rotor height " << height << ": airframe rotor " << i
                                                     << " (" << part.name << "), axis " << a;
            }
        }
        for (std::size_t i = 0; i < kQuadrotorRotorCount; ++i) {
            EXPECT_EQ(props[i], 1) << "rotor height " << height << ": airframe rotor " << i << "'s props";
        }
    }
}

TEST(QuadrotorLook, KeepsEveryRotorsRadiusAndSpinAndTheHandFollowsIt) {
    const std::array<std::array<float, kQuadrotorRotorCount>, 2> layouts = {
        vh::kQuadrotorSpinLayout, std::array<float, kQuadrotorRotorCount>{1.0f, 1.0f, -1.0f, -1.0f}};
    for (const auto& spins : layouts) {
        vh::QuadrotorParams airframe = stand();
        airframe.spin_dirs = spins;
        const spade::Result<vh::QuadrotorLook> look = vh::quadrotor_look(airframe, base(), blade());
        ASSERT_TRUE(look.has_value()) << look.error().context;
        ASSERT_EQ(look->model.rotors.size(), kQuadrotorRotorCount);
        for (std::size_t i = 0; i < kQuadrotorRotorCount; ++i) {
            const vh::RotorDesc& r = look->model.rotors[look->rotor[i]];
            EXPECT_EQ(r.radius, airframe.rotors[i].radius) << "airframe rotor " << i;
            EXPECT_EQ(r.spin_dir, airframe.spin_dirs[i]) << "airframe rotor " << i;
        }
        // The spin each prop shows (s_k omega along the thrust axis, +Y here),
        // and its hand: one mesh per spin.
        const spade::Result<vh::PosedParts> parts = posed(*look, 1.0f);
        ASSERT_TRUE(parts.has_value()) << parts.error().context;
        std::array<MeshPtr, 2> hand{};  // [0] spin +1, [1] spin -1
        for (const vh::PosedPart& pp : parts->parts) {
            const vh::PartDesc& part = look->model.parts[pp.part];
            if (part.attach != vh::PartAttach::rotor) continue;
            const std::size_t i = airframe_rotor(*look, part.rotor);
            ASSERT_LT(i, kQuadrotorRotorCount);
            EXPECT_NEAR(pp.spin.y, airframe.spin_dirs[i], 1e-5f) << part.name;
            ASSERT_NE(part.mesh, nullptr) << part.name;
            MeshPtr& h = hand[airframe.spin_dirs[i] > 0.0f ? 0 : 1];
            if (h == nullptr) h = part.mesh;
            EXPECT_EQ(part.mesh, h) << part.name << ": one mesh per hand";
        }
        EXPECT_NE(hand[0], hand[1]) << "the two hands are two meshes";
    }
}

TEST(QuadrotorLook, DrawsABodyFourArmsFourBellsAndFourPropsAsMeshes) {
    const spade::Result<vh::QuadrotorLook> look = vh::quadrotor_look(stand(), base(), blade());
    ASSERT_TRUE(look.has_value()) << look.error().context;
    int body = 0, arms = 0, bells = 0, props = 0;
    for (const vh::PartDesc& part : look->model.parts) {
        const bool counted = part.role == "body" || part.role == "arm" || part.role == "motor" || part.role == "prop";
        if (!counted) continue;
        EXPECT_NE(part.mesh, nullptr) << part.name << " is drawn as a mesh";
        body += part.role == "body" ? 1 : 0;
        arms += part.role == "arm" ? 1 : 0;
        bells += part.role == "motor" ? 1 : 0;
        props += part.role == "prop" ? 1 : 0;
    }
    EXPECT_EQ(body, 1);
    EXPECT_EQ(arms, 4);
    EXPECT_EQ(bells, 4);
    EXPECT_EQ(props, 4);
}

TEST(QuadrotorLook, RefusesRotorsOfDifferentRadiiByName) {
    vh::QuadrotorParams airframe = stand();
    airframe.rotors[2].radius = 0.11f;
    const spade::Result<vh::QuadrotorLook> look = vh::quadrotor_look(airframe, base(), blade());
    ASSERT_FALSE(look.has_value());
    EXPECT_EQ(look.error().code, spade::Code::invalid_argument);
    EXPECT_NE(look.error().context.find("rotors[2].radius"), std::string::npos) << look.error().context;
}

TEST(QuadrotorLook, PassesMakeQuadrotorsRefusalOnVerbatim) {
    vh::QuadrotorParams airframe = stand();
    airframe.arm_length = 0.0f;
    const spade::Result<vh::ModelType> flown = vh::make_quadrotor(airframe);
    ASSERT_FALSE(flown.has_value());
    const spade::Result<vh::QuadrotorLook> look = vh::quadrotor_look(airframe, base(), blade());
    ASSERT_FALSE(look.has_value());
    EXPECT_EQ(look.error().code, flown.error().code);
    EXPECT_EQ(look.error().context, flown.error().context);
}

// RED until Physics' defaults fix merges (the lead, 2026-11-03): make_quadrotor()
// refusing overlapping discs by name is the one rule the look passes on. The
// stand at 0.15 m arms puts its neighbours 212 mm apart under 240 mm discs.
TEST(QuadrotorLook, RefusesOverlappingDiscsByMakeQuadrotorsRule) {
    vh::QuadrotorParams airframe = stand();
    airframe.arm_length = 0.15f;
    const spade::Result<vh::QuadrotorLook> look = vh::quadrotor_look(airframe, base(), blade());
    ASSERT_FALSE(look.has_value()) << "overlapping discs were drawn";
    EXPECT_EQ(look.error().code, spade::Code::invalid_argument);
    EXPECT_NE(look.error().context.find("overlap"), std::string::npos) << look.error().context;
}

}  // namespace
