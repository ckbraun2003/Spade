#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>

#include "vehicles/composite_inertia.hpp"

// ===========================================================================
// The drone builder's composite-inertia utility
// (vehicles/composite_inertia.hpp). Every expectation is a textbook moment
// or the parallel-axis theorem, computed here in double (TD-4).
// ===========================================================================

using namespace spade::vehicles;

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

[[nodiscard]] PartInertia part(PartShape shape, double mass, glm::dvec3 pos, glm::dvec3 size = glm::dvec3(0.0),
                               glm::dquat orient = glm::dquat(1.0, 0.0, 0.0, 0.0)) {
    PartInertia p;
    p.shape = shape;
    p.mass = mass;
    p.position = pos;
    p.size = size;
    p.orientation = orient;
    return p;
}

// A plus-layout quadrotor in the design frame (Y up, nose +X): a central box,
// four arms as tubes, and four motors as cylinders at +X, +Z, -X, -Z.
[[nodiscard]] std::vector<PartInertia> symmetric_quad() {
    const double arm = 0.15;
    const glm::dquat along_x = glm::angleAxis(glm::half_pi<double>(), glm::dvec3(0.0, 0.0, 1.0));  // +Y -> -X
    const glm::dquat along_z = glm::angleAxis(glm::half_pi<double>(), glm::dvec3(1.0, 0.0, 0.0));  // +Y -> +Z
    std::vector<PartInertia> parts;
    parts.push_back(part(PartShape::box, 0.30, glm::dvec3(0.0), glm::dvec3(0.10, 0.04, 0.08)));
    parts.push_back(part(PartShape::tube, 0.02, glm::dvec3(+arm / 2, 0.0, 0.0), glm::dvec3(0.008, arm, 0.0), along_x));
    parts.push_back(part(PartShape::tube, 0.02, glm::dvec3(-arm / 2, 0.0, 0.0), glm::dvec3(0.008, arm, 0.0), along_x));
    parts.push_back(part(PartShape::tube, 0.02, glm::dvec3(0.0, 0.0, +arm / 2), glm::dvec3(0.008, arm, 0.0), along_z));
    parts.push_back(part(PartShape::tube, 0.02, glm::dvec3(0.0, 0.0, -arm / 2), glm::dvec3(0.008, arm, 0.0), along_z));
    for (const glm::dvec3 at : {glm::dvec3(arm, 0.02, 0.0), glm::dvec3(0.0, 0.02, arm), glm::dvec3(-arm, 0.02, 0.0),
                                glm::dvec3(0.0, 0.02, -arm)}) {
        parts.push_back(part(PartShape::cylinder, 0.033, at, glm::dvec3(0.014, 0.018, 0.0)));
    }
    return parts;
}

}  // namespace

TEST(CompositeInertia, EachPrimitiveMatchesItsTextbookMoments) {
    const double m = 2.0;
    glm::dmat3 i = part_inertia_local(part(PartShape::sphere, m, glm::dvec3(0.0), glm::dvec3(0.1, 0.0, 0.0)));
    EXPECT_DOUBLE_EQ(i[0][0], 2.0 / 5.0 * m * 0.01);
    EXPECT_DOUBLE_EQ(i[1][1], 2.0 / 5.0 * m * 0.01);

    i = part_inertia_local(part(PartShape::box, m, glm::dvec3(0.0), glm::dvec3(0.2, 0.4, 0.6)));
    EXPECT_DOUBLE_EQ(i[0][0], m * (0.16 + 0.36) / 12.0);
    EXPECT_DOUBLE_EQ(i[1][1], m * (0.04 + 0.36) / 12.0);
    EXPECT_DOUBLE_EQ(i[2][2], m * (0.04 + 0.16) / 12.0);

    i = part_inertia_local(part(PartShape::cylinder, m, glm::dvec3(0.0), glm::dvec3(0.1, 0.5, 0.0)));
    EXPECT_DOUBLE_EQ(i[1][1], m * 0.01 / 2.0) << "about its own axis, local +Y";
    EXPECT_DOUBLE_EQ(i[0][0], m * (3.0 * 0.01 + 0.25) / 12.0);

    i = part_inertia_local(part(PartShape::tube, m, glm::dvec3(0.0), glm::dvec3(0.1, 0.5, 0.0)));
    EXPECT_DOUBLE_EQ(i[1][1], m * 0.01) << "a thin wall: all the mass at radius r";
    EXPECT_DOUBLE_EQ(i[0][0], m * (0.01 / 2.0 + 0.25 / 12.0));

    i = part_inertia_local(part(PartShape::point, m, glm::dvec3(1.0)));
    EXPECT_EQ(i, glm::dmat3(0.0));
}

TEST(CompositeInertia, PointMassesFollowTheParallelAxisTheorem) {
    // A 1 kg sphere at the origin and two 0.5 kg points at x = +-0.2.
    const std::vector<PartInertia> parts = {
        part(PartShape::sphere, 1.0, glm::dvec3(0.0), glm::dvec3(0.05, 0.0, 0.0)),
        part(PartShape::point, 0.5, glm::dvec3(+0.2, 0.0, 0.0)),
        part(PartShape::point, 0.5, glm::dvec3(-0.2, 0.0, 0.0)),
    };
    const auto c = composite_inertia(parts);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    const double sphere = 0.4 * 1.0 * 0.05 * 0.05;
    const double points = 2.0 * 0.5 * 0.2 * 0.2;
    EXPECT_EQ(c->mass, 2.0f);
    EXPECT_EQ(c->center_of_mass, glm::vec3(0.0f));
    EXPECT_EQ(c->principal_moments.x, static_cast<float>(sphere));
    EXPECT_FLOAT_EQ(c->principal_moments.y, static_cast<float>(sphere + points));
    EXPECT_FLOAT_EQ(c->principal_moments.z, static_cast<float>(sphere + points));
}

TEST(CompositeInertia, ARotatedPartRotatesItsTensor) {
    // A box 0.6 long along local x, turned 90 degrees about y: now long along z.
    const glm::dquat turn = glm::angleAxis(glm::half_pi<double>(), glm::dvec3(0.0, 1.0, 0.0));
    const std::vector<PartInertia> parts = {
        part(PartShape::box, 1.0, glm::dvec3(0.0), glm::dvec3(0.6, 0.1, 0.2), turn)};
    const auto c = composite_inertia(parts);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    // In the design frame the extents are (0.2, 0.1, 0.6) along x, y, z.
    EXPECT_NEAR(c->inertia_design[0][0], (0.01 + 0.36) / 12.0, 1e-15);
    EXPECT_NEAR(c->inertia_design[2][2], (0.04 + 0.01) / 12.0, 1e-15);
    EXPECT_NEAR(c->inertia_design[1][1], (0.04 + 0.36) / 12.0, 1e-15);
}

TEST(CompositeInertia, ASymmetricAirframeKeepsItsDesignAxesExactly) {
    const std::vector<PartInertia> parts = symmetric_quad();
    const auto c = composite_inertia(parts);
    ASSERT_TRUE(c.has_value()) << c.error().context;
    EXPECT_EQ(c->design_to_body.w, 1.0f);
    EXPECT_EQ(c->design_to_body.x, 0.0f);
    EXPECT_EQ(c->design_to_body.y, 0.0f);
    EXPECT_EQ(c->design_to_body.z, 0.0f);
    // The moments are the design-frame diagonal, rounded once.
    EXPECT_EQ(c->principal_moments.x, static_cast<float>(c->inertia_design[0][0]));
    EXPECT_EQ(c->principal_moments.y, static_cast<float>(c->inertia_design[1][1]));
    EXPECT_EQ(c->principal_moments.z, static_cast<float>(c->inertia_design[2][2]));
    // Mass is the double sum, rounded once.
    double m = 0.0;
    for (const PartInertia& p : parts) m += p.mass;
    EXPECT_EQ(c->mass, static_cast<float>(m));
}

TEST(CompositeInertia, AnAsymmetricAirframeIsDiagonalizedByARightHandedRotation) {
    std::vector<PartInertia> parts = symmetric_quad();
    // An offset battery, tilted 20 degrees about an oblique axis.
    const glm::dquat tilt = glm::angleAxis(glm::radians(20.0), glm::normalize(glm::dvec3(1.0, 2.0, 0.5)));
    parts.push_back(part(PartShape::box, 0.18, glm::dvec3(0.03, -0.03, 0.01), glm::dvec3(0.07, 0.03, 0.035), tilt));
    const auto c = composite_inertia(parts);
    ASSERT_TRUE(c.has_value()) << c.error().context;

    // R_bd I R_bd^T must be diagonal, with the published moments on its diagonal.
    const glm::dmat3 r = glm::mat3_cast(glm::dquat(c->design_to_body.w, c->design_to_body.x,
                                                   c->design_to_body.y, c->design_to_body.z));
    const glm::dmat3 body = r * c->inertia_design * glm::transpose(r);
    const double scale = body[0][0] + body[1][1] + body[2][2];
    for (int col = 0; col < 3; ++col)
        for (int row = 0; row < 3; ++row) {
            if (row == col) continue;
            EXPECT_LT(std::fabs(body[col][row]), 1e-6 * scale) << "off-diagonal (" << row << ", " << col << ")";
        }
    EXPECT_NEAR(body[0][0], c->principal_moments.x, 1e-6 * scale);
    EXPECT_NEAR(body[1][1], c->principal_moments.y, 1e-6 * scale);
    EXPECT_NEAR(body[2][2], c->principal_moments.z, 1e-6 * scale);
    // Each body axis keeps its design label: R_bd's diagonal dominates its row.
    for (int k = 0; k < 3; ++k) {
        EXPECT_GT(r[k][k], 0.9) << "body axis " << k << " strays from design axis " << k;
    }
}

TEST(CompositeInertia, RefusesInvalidParts) {
    EXPECT_FALSE(composite_inertia({}).has_value()) << "no parts";
    const std::vector<PartInertia> ok = {part(PartShape::sphere, 1.0, glm::dvec3(0.0), glm::dvec3(0.1, 0.0, 0.0))};

    std::vector<PartInertia> bad = ok;
    bad[0].mass = 0.0;
    EXPECT_FALSE(composite_inertia(bad).has_value()) << "zero mass";
    bad = ok;
    bad[0].position.y = kNaN;
    EXPECT_FALSE(composite_inertia(bad).has_value()) << "non-finite position";
    bad = ok;
    bad[0].orientation = glm::dquat(0.0, 0.0, 0.0, 0.0);
    EXPECT_FALSE(composite_inertia(bad).has_value()) << "zero orientation";
    bad = ok;
    bad[0].size.x = 0.0;
    EXPECT_FALSE(composite_inertia(bad).has_value()) << "a sphere with no radius";

    bad = ok;
    bad[0].shape = PartShape::tensor;
    bad[0].tensor = glm::dmat3(1.0);
    bad[0].tensor[0][1] = 0.1;  // column 0, row 1 -- but not row 0, column 1
    EXPECT_FALSE(composite_inertia(bad).has_value()) << "an asymmetric tensor";
    bad[0].tensor = glm::dmat3(1.0);
    bad[0].tensor[2][2] = 3.0;
    EXPECT_FALSE(composite_inertia(bad).has_value()) << "1 + 1 < 3 breaks the triangle inequality";
    bad[0].tensor = glm::dmat3(1.0);
    bad[0].tensor[1][1] = -0.5;
    EXPECT_FALSE(composite_inertia(bad).has_value()) << "a negative moment";

    // Point masses on one line have no moment about it: the engine cannot
    // store that body's inverse inertia.
    const std::vector<PartInertia> rod = {part(PartShape::point, 1.0, glm::dvec3(+0.1, 0.0, 0.0)),
                                          part(PartShape::point, 1.0, glm::dvec3(-0.1, 0.0, 0.0))};
    EXPECT_FALSE(composite_inertia(rod).has_value());
}
