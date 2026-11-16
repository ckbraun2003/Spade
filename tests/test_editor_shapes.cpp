// L82, the editor half: a model's contact shape (L68), display-free. The checks the inspector makes before an edit
// (the engine's own refusals, and the editor's Q7), the hull the "From model" button builds, the inertia the "Inertia
// from box" button sets, the frames the wireframe is drawn in, and the wireframe's segments.
//
// Expectations are hand-written: the half extents, counts and the numbers of the formulas worked on paper.

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include <glm/gtc/quaternion.hpp>

#include "../sandbox/editor_shapes.hpp"
#include "editor_test_support.hpp"
#include "physics/rigid_contact.hpp"
#include "vehicles/model_type.hpp"

namespace {

using spade::physics::ShapeDesc;
using spade::sandbox::editor::body_pose;
using spade::sandbox::editor::box_inertia_diag;
using spade::sandbox::editor::check_shape;
using spade::sandbox::editor::hull_from_model;
using spade::sandbox::editor::Segment;
using spade::sandbox::editor::shape_segments;
using spade::sandbox::editor::shape_for_kind;
using spade::sandbox::editor::inertia_from_box;
using spade::sandbox::editor::test::quad_model;

// The quad with its design frame made trivial, one body part (a box of half extents 0.08, 0.03, 0.08) and its four
// rotors on the diagonals at (+-0.2, 0, +-0.2), radius 0.1.
spade::vehicles::ModelType quad_with_a_body() {
    spade::vehicles::ModelType m = quad_model();
    m.design_to_principal = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    m.com_offset = glm::vec3(0.0f);
    m.parts.clear();
    spade::vehicles::PartDesc body;
    body.name = "body";
    body.shape.transforms.push_back(spade::SdfTransform{});
    spade::SdfNode box{};
    box.kind = static_cast<uint32_t>(spade::SdfPrim::box);
    box.op = static_cast<uint32_t>(spade::SdfOp::none);
    box.params = glm::vec4(0.08f, 0.03f, 0.08f, 0.0f);
    body.shape.nodes.push_back(box);
    body.min_feature = 0.01f;
    m.parts.push_back(body);
    for (std::size_t i = 0; i < m.rotors.size(); ++i) {
        m.rotors[i].local_pos = glm::vec3((i & 1) ? 0.2f : -0.2f, 0.0f, (i & 2) ? 0.2f : -0.2f);
        m.rotors[i].radius = 0.1f;
    }
    return m;
}

TEST(EditorShapes, TheEngineRefusalsReachTheInspectorByName) {
    const spade::vehicles::ModelType m = quad_with_a_body();
    EXPECT_TRUE(check_shape(m, spade::physics::sphere_shape(0.1f)).has_value());
    const auto zero = check_shape(m, spade::physics::sphere_shape(0.0f));
    ASSERT_FALSE(zero.has_value());
    EXPECT_NE(zero.error().context.find("set_body_shape:"), std::string::npos) << zero.error().context;
    EXPECT_NE(zero.error().context.find(m.name), std::string::npos) << "the refusal names the model";

    ShapeDesc too_many;
    too_many.kind = spade::physics::ShapeKind::hull;
    for (int i = 0; i < 17; ++i) too_many.points.emplace_back(static_cast<float>(i), 0.0f, 0.0f);
    const auto refused = check_shape(m, too_many);
    ASSERT_FALSE(refused.has_value());
    EXPECT_NE(refused.error().context.find("16"), std::string::npos) << refused.error().context;
}

TEST(EditorShapes, ABoxNeedsAnAxisAlignedDesignFrame) {
    spade::vehicles::ModelType m = quad_with_a_body();
    const ShapeDesc box = spade::physics::box_shape(glm::vec3(0.1f, 0.2f, 0.3f));
    EXPECT_TRUE(check_shape(m, box).has_value());

    m.design_to_principal = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    EXPECT_TRUE(check_shape(m, box).has_value()) << "a quarter turn only permutes the axes";

    m.design_to_principal = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const auto turned = check_shape(m, box);
    ASSERT_FALSE(turned.has_value());
    EXPECT_NE(turned.error().context.find("design_to_principal"), std::string::npos) << turned.error().context;
    EXPECT_NE(turned.error().context.find("multiple of 90"), std::string::npos) << turned.error().context;
    // Only a box is refused: the other kinds have no axes to permute.
    EXPECT_TRUE(check_shape(m, spade::physics::sphere_shape(0.1f)).has_value());
    EXPECT_TRUE(check_shape(m, spade::physics::capsule_shape(glm::vec3(-0.2f, 0, 0), glm::vec3(0.2f, 0, 0), 0.05f))
                    .has_value());
}

TEST(EditorShapes, InertiaFromBoxIsTheSolidBoxsMoments) {
    // m = 2, half extents (0.1, 0.2, 0.3): I_x = m/3 (hy^2 + hz^2) = 2/3 (0.04 + 0.09), and so on.
    const glm::vec3 i = box_inertia_diag(2.0f, glm::vec3(0.1f, 0.2f, 0.3f));
    EXPECT_NEAR(i.x, 0.086667f, 1e-5f);
    EXPECT_NEAR(i.y, 0.066667f, 1e-5f);
    EXPECT_NEAR(i.z, 0.033333f, 1e-5f);
    const glm::vec3 inv = spade::physics::box_inv_inertia(2.0f, glm::vec3(0.1f, 0.2f, 0.3f));
    EXPECT_NEAR(i.x * inv.x, 1.0f, 1e-5f);
    EXPECT_NEAR(i.y * inv.y, 1.0f, 1e-5f);
    EXPECT_NEAR(i.z * inv.z, 1.0f, 1e-5f);
}

TEST(EditorShapes, TheInertiaButtonOffersTheBoxsMomentsForTheModelsMassOnly) {
    spade::vehicles::ModelType m = quad_with_a_body();
    m.body.mass = 2.0f;
    EXPECT_FALSE(inertia_from_box(m).has_value()) << "no shape, no button";
    m.shape = spade::physics::sphere_shape(0.1f);
    EXPECT_FALSE(inertia_from_box(m).has_value()) << "only a box";
    m.shape = spade::physics::box_shape(glm::vec3(0.1f, 0.2f, 0.3f));
    const auto i = inertia_from_box(m);
    ASSERT_TRUE(i.has_value());
    EXPECT_NEAR(i->x, 0.086667f, 1e-5f);  // box_inertia_diag(2, (0.1, 0.2, 0.3)) from above
    EXPECT_NEAR(i->y, 0.066667f, 1e-5f);
    EXPECT_NEAR(i->z, 0.033333f, 1e-5f);
}

TEST(EditorShapes, TheHullFromAModelIsItsBodysEightCornersAndItsRotorTips) {
    const spade::vehicles::ModelType m = quad_with_a_body();
    const auto hull = hull_from_model(m);
    ASSERT_TRUE(hull.has_value()) << (hull ? "" : hull.error().context);
    EXPECT_EQ(hull->kind, spade::physics::ShapeKind::hull);
    ASSERT_EQ(hull->points.size(), 12u);  // 8 corners + 4 rotor tips

    // The corners are the body part's box: (+-0.08, +-0.03, +-0.08), each once.
    int corners = 0;
    for (const glm::vec3& p : hull->points) {
        if (std::fabs(std::fabs(p.x) - 0.08f) < 1e-4f && std::fabs(std::fabs(p.y) - 0.03f) < 1e-4f &&
            std::fabs(std::fabs(p.z) - 0.08f) < 1e-4f) {
            ++corners;
        }
    }
    EXPECT_EQ(corners, 8);
    // A rotor tip is its hub pushed out along the line from the centre of mass by the rotor's radius: the hub is 0.28284
    // m out, the tip 0.38284 m, so each coordinate is 0.2 * 0.38284 / 0.28284 = 0.270711.
    int tips = 0;
    for (const glm::vec3& p : hull->points) {
        if (std::fabs(std::fabs(p.x) - 0.270711f) < 1e-4f && std::fabs(p.y) < 1e-4f &&
            std::fabs(std::fabs(p.z) - 0.270711f) < 1e-4f) {
            ++tips;
        }
    }
    EXPECT_EQ(tips, 4);
}

TEST(EditorShapes, TheHullIsInTheBodyFrame) {
    // The centre of mass 0.05 m up the design y: a design point is the body point less that, since the body's origin is the CoM.
    spade::vehicles::ModelType m = quad_with_a_body();
    m.com_offset = glm::vec3(0.0f, 0.05f, 0.0f);
    const auto hull = hull_from_model(m);
    ASSERT_TRUE(hull.has_value()) << (hull ? "" : hull.error().context);
    float lowest = 1e9f;
    float highest = -1e9f;
    for (const glm::vec3& p : hull->points) {
        lowest = std::min(lowest, p.y);
        highest = std::max(highest, p.y);
    }
    EXPECT_NEAR(highest, 0.03f - 0.05f, 1e-5f);  // the body's top corners: 0.03 in design, -0.02 from the CoM
    EXPECT_NEAR(lowest, -0.03f - 0.05f, 1e-5f);
}

TEST(EditorShapes, AHullThatNeedsMoreThanSixteenPointsIsRefusedByName) {
    spade::vehicles::ModelType m = quad_with_a_body();
    while (m.rotors.size() < 9) m.rotors.push_back(m.rotors.front());  // 8 corners + 9 tips = 17
    const auto hull = hull_from_model(m);
    ASSERT_FALSE(hull.has_value());
    EXPECT_NE(hull.error().context.find("17"), std::string::npos) << hull.error().context;
    EXPECT_NE(hull.error().context.find("16"), std::string::npos) << hull.error().context;
    EXPECT_NE(hull.error().context.find(m.name), std::string::npos) << hull.error().context;
}

TEST(EditorShapes, AModelWithNoPartsHasNoHull) {
    spade::vehicles::ModelType m = quad_with_a_body();
    m.parts.clear();
    const auto hull = hull_from_model(m);
    ASSERT_FALSE(hull.has_value());
    EXPECT_NE(hull.error().context.find("no parts"), std::string::npos) << hull.error().context;
}

TEST(EditorShapes, TheBodyPoseIsTheEngines) {
    // spade::to_body_state is the one conversion (design_frame.hpp): p_com = p_d + R c, q_wb = q_wd * conj(q_bd). A design
    // origin at (1, 2, 3), the identity orientation and the centre of mass 0.1 m up: the body is at (1, 2.1, 3).
    spade::vehicles::ModelType m = quad_with_a_body();
    m.com_offset = glm::vec3(0.0f, 0.1f, 0.0f);
    const spade::FrameState b = body_pose(m, glm::vec3(1.0f, 2.0f, 3.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    EXPECT_NEAR(b.pos.x, 1.0f, 1e-6f);
    EXPECT_NEAR(b.pos.y, 2.1f, 1e-6f);
    EXPECT_NEAR(b.pos.z, 3.0f, 1e-6f);
}

TEST(EditorShapes, PickingAKindGivesDefaultsFromTheBodyPartsBox) {
    using spade::physics::ShapeKind;
    const spade::vehicles::ModelType m = quad_with_a_body();  // the body's box: half extents (0.08, 0.03, 0.08), centred on the CoM
    const auto box = shape_for_kind(m, ShapeKind::box);
    ASSERT_TRUE(box.has_value()) << (box ? "" : box.error().context);
    EXPECT_EQ(box->kind, ShapeKind::box);
    EXPECT_NEAR(box->half_extents.x, 0.08f, 1e-5f);
    EXPECT_NEAR(box->half_extents.y, 0.03f, 1e-5f);
    EXPECT_NEAR(box->half_extents.z, 0.08f, 1e-5f);

    const auto sphere = shape_for_kind(m, ShapeKind::sphere);
    ASSERT_TRUE(sphere.has_value());
    EXPECT_NEAR(sphere->radius, 0.08f, 1e-5f);  // the largest half extent: it holds the box

    // The capsule lies along the longest axis (x, the first of the two equal ones); its radius is the smaller of the other two
    // half extents (0.03), its ends at +-(0.08 - 0.03).
    const auto capsule = shape_for_kind(m, ShapeKind::capsule);
    ASSERT_TRUE(capsule.has_value());
    EXPECT_NEAR(capsule->radius, 0.03f, 1e-5f);
    EXPECT_NEAR(capsule->a.x, -0.05f, 1e-5f);
    EXPECT_NEAR(capsule->b.x, 0.05f, 1e-5f);
    EXPECT_NEAR(capsule->a.y, 0.0f, 1e-6f);
    EXPECT_NEAR(capsule->b.z, 0.0f, 1e-6f);

    const auto hull = shape_for_kind(m, ShapeKind::hull);
    ASSERT_TRUE(hull.has_value());
    EXPECT_EQ(hull->points.size(), 12u);

    const auto none = shape_for_kind(m, ShapeKind::none);
    ASSERT_TRUE(none.has_value());
    EXPECT_EQ(none->kind, ShapeKind::none);
}

TEST(EditorShapes, TheDefaultsAreInTheBodyFrameAndHoldTheBoxFromTheCentreOfMass) {
    // The centre of mass 0.05 m up: the part's box spans y in [-0.03, 0.03] in the design, which is [-0.08, -0.02] from the CoM, so
    // the half extent that holds it from there is 0.08.
    spade::vehicles::ModelType m = quad_with_a_body();
    m.com_offset = glm::vec3(0.0f, 0.05f, 0.0f);
    const auto box = shape_for_kind(m, spade::physics::ShapeKind::box);
    ASSERT_TRUE(box.has_value());
    EXPECT_NEAR(box->half_extents.y, 0.08f, 1e-5f);

    // A quarter turn about y swaps x and z: a part box of half extents (0.08, 0.03, 0.05) is (0.05, 0.03, 0.08) in the body.
    spade::vehicles::ModelType turned = quad_with_a_body();
    turned.parts.at(0).shape.nodes.at(0).params = glm::vec4(0.08f, 0.03f, 0.05f, 0.0f);
    turned.design_to_principal = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    const auto swapped = shape_for_kind(turned, spade::physics::ShapeKind::box);
    ASSERT_TRUE(swapped.has_value()) << (swapped ? "" : swapped.error().context);
    EXPECT_NEAR(swapped->half_extents.x, 0.05f, 1e-4f);
    EXPECT_NEAR(swapped->half_extents.y, 0.03f, 1e-4f);
    EXPECT_NEAR(swapped->half_extents.z, 0.08f, 1e-4f);
}

TEST(EditorShapes, AModelWithNoPartsGetsPlainDefaultsExceptTheHull) {
    using spade::physics::ShapeKind;
    spade::vehicles::ModelType m = quad_with_a_body();
    m.parts.clear();
    EXPECT_NEAR(shape_for_kind(m, ShapeKind::sphere)->radius, 0.1f, 1e-6f);
    EXPECT_NEAR(shape_for_kind(m, ShapeKind::box)->half_extents.x, 0.1f, 1e-6f);
    const auto capsule = shape_for_kind(m, ShapeKind::capsule);
    ASSERT_TRUE(capsule.has_value());
    EXPECT_NEAR(capsule->radius, 0.05f, 1e-6f);
    EXPECT_NEAR(capsule->b.x - capsule->a.x, 0.2f, 1e-6f);
    EXPECT_FALSE(shape_for_kind(m, ShapeKind::hull).has_value());  // a hull needs parts: it says so
}

TEST(EditorShapes, TheWireframesHaveTheirDocumentedSegments) {
    // Box: twelve edges, each as long as an edge: 0.2, 0.4 or 0.6 for half extents (0.1, 0.2, 0.3).
    const std::vector<Segment> box = shape_segments(spade::physics::box_shape(glm::vec3(0.1f, 0.2f, 0.3f)));
    ASSERT_EQ(box.size(), 12u);
    int along[3] = {0, 0, 0};
    for (const Segment& s : box) {
        const float len = glm::length(s.b - s.a);
        if (std::fabs(len - 0.2f) < 1e-5f) ++along[0];
        if (std::fabs(len - 0.4f) < 1e-5f) ++along[1];
        if (std::fabs(len - 0.6f) < 1e-5f) ++along[2];
    }
    EXPECT_EQ(along[0], 4);
    EXPECT_EQ(along[1], 4);
    EXPECT_EQ(along[2], 4);

    // Sphere: three great circles of 24 segments, every vertex on the sphere.
    const std::vector<Segment> sphere = shape_segments(spade::physics::sphere_shape(0.25f));
    ASSERT_EQ(sphere.size(), 72u);
    for (const Segment& s : sphere) {
        EXPECT_NEAR(glm::length(s.a), 0.25f, 1e-5f);
        EXPECT_NEAR(glm::length(s.b), 0.25f, 1e-5f);
    }

    // Capsule: two end circles of 24 segments, four side lines, between the ends.
    const std::vector<Segment> capsule =
        shape_segments(spade::physics::capsule_shape(glm::vec3(-0.3f, 0, 0), glm::vec3(0.3f, 0, 0), 0.05f));
    EXPECT_EQ(capsule.size(), 2u * 24u + 4u);

    // Hull: a box's eight corners as a hull draw the box's twelve edges and nothing across a face.
    ShapeDesc corners = spade::physics::hull_shape({});
    for (int c = 0; c < 8; ++c) {
        corners.points.emplace_back((c & 1) ? 0.1f : -0.1f, (c & 2) ? 0.2f : -0.2f, (c & 4) ? 0.3f : -0.3f);
    }
    EXPECT_EQ(shape_segments(corners).size(), 12u);

    // None draws nothing.
    EXPECT_TRUE(shape_segments(ShapeDesc{}).empty());
}

}  // namespace
