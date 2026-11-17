// The dye released at a line's blade tips (L86i; physics/airflow/dye.hpp's tip_points() and emit_tip_dye()): the points are where the
// blades' tips are at the step's sub-azimuths, the release is a 3D Peskin kernel about each. Expectations are worked by hand (TD-4).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "physics/airflow/blade.hpp"
#include "physics/airflow/dye.hpp"
#include "physics/airflow/line.hpp"
#include "physics/airflow/mac_grid.hpp"

namespace {

namespace af = spade::physics::airflow;

constexpr float kPi = std::numbers::pi_v<float>;

af::BladeShape blade_of(uint32_t blades, float radius) {
    af::BladeShape b;
    b.blades = blades;
    b.radius = radius;
    b.stations = {{0.15f * radius, 0.1f * radius, 30.0f}, {0.6f * radius, 0.09f * radius, 18.0f}, {radius, 0.05f * radius, 10.0f}};
    return b;
}

af::LineRotor rotor_at(glm::vec3 centre, float spin, float omega, float psi, const af::BladeShape& blade,
                       const af::PolarTable& polar) {
    af::LineRotor r;
    r.centre = centre;
    r.axis = {0.0f, 1.0f, 0.0f};
    r.ref = {1.0f, 0.0f, 0.0f};
    r.spin = spin;
    r.omega = omega;
    r.psi = psi;
    r.blade = &blade;
    r.polar = &polar;
    return r;
}

// The point of blade b at sub-azimuth k, by hand: the line's rule phi = s (psi + omega h (k + 1/2) / n_sub) + 2 pi b / B, the blade's
// direction ref cos phi + (axis x ref) sin phi, the point frac R along it from the centre.
glm::vec3 by_hand(const af::LineRotor& r, float h, uint32_t n_sub, uint32_t k, uint32_t b, float frac) {
    const double psi_k = static_cast<double>(r.psi) +
                         static_cast<double>(r.omega) * h * (static_cast<double>(k) + 0.5) / static_cast<double>(n_sub);
    const double phi = static_cast<double>(r.spin) * psi_k + 2.0 * std::numbers::pi * b / r.blade->blades;
    const glm::dvec3 side = glm::cross(glm::dvec3(r.axis), glm::dvec3(r.ref));
    const glm::dvec3 d = glm::dvec3(r.ref) * std::cos(phi) + side * std::sin(phi);
    return glm::vec3(glm::dvec3(r.centre) + d * (static_cast<double>(frac) * r.blade->radius));
}

TEST(AirflowTipDye, ThePointsAreWhereTheBladeTipsAreAtTheSubAzimuths) {
    const af::BladeShape blade = blade_of(2, 0.1194f);
    const af::PolarTable polar = af::estimated_polar();
    const float h = 0.0002f;
    const uint32_t n_sub = 8;
    for (const float spin : {1.0f, -1.0f}) {
        const af::LineRotor r = rotor_at({0.5f, 0.6f, 0.4f}, spin, 555.0f, 0.9f, blade, polar);
        const std::vector<af::LineRotor> rotors{r};
        const std::vector<glm::vec3> pts = af::tip_points(rotors, 0.004f, h, n_sub, 1.0f);
        ASSERT_EQ(pts.size(), std::size_t{n_sub} * 2u) << "a point a blade a sub-azimuth";
        std::size_t i = 0;
        for (uint32_t k = 0; k < n_sub; ++k) {
            for (uint32_t b = 0; b < 2; ++b, ++i) {
                const glm::vec3 want = by_hand(r, h, n_sub, k, b, 1.0f);
                for (int a = 0; a < 3; ++a) EXPECT_NEAR(pts[i][a], want[a], 2.0e-5f) << "spin " << spin << " k " << k << " b " << b;
            }
        }
    }
}

TEST(AirflowTipDye, TheReleaseFollowsTheBladesAzimuthAndTheFraction) {
    const af::BladeShape blade = blade_of(3, 0.1f);
    const af::PolarTable polar = af::estimated_polar();
    const float h = 0.0002f;
    const glm::vec3 centre{0.4f, 0.5f, 0.45f};
    const af::LineRotor r0 = rotor_at(centre, 1.0f, 600.0f, 0.2f, blade, polar);
    af::LineRotor r1 = r0;
    r1.psi = r0.psi + 0.5f;  // the blades have turned half a radian
    const std::vector<glm::vec3> p0 = af::tip_points(std::vector<af::LineRotor>{r0}, 0.004f, h, 4, 0.9f);
    const std::vector<glm::vec3> p1 = af::tip_points(std::vector<af::LineRotor>{r1}, 0.004f, h, 4, 0.9f);
    ASSERT_EQ(p0.size(), 12u);
    ASSERT_EQ(p1.size(), 12u);
    for (std::size_t i = 0; i < p0.size(); ++i) {
        const glm::vec3 a = p0[i] - centre, b = p1[i] - centre;
        EXPECT_NEAR(std::sqrt(a.x * a.x + a.z * a.z), 0.09f, 2.0e-6f) << "frac 0.9 of R";
        EXPECT_NEAR(a.y, 0.0f, 1.0e-6f) << "in the rotor's plane";
        // Spin +1 about +y with ref +x, side = axis x ref = (0, 0, -1): phi grows from +x toward -z.
        const float phi0 = std::atan2(-a.z, a.x), phi1 = std::atan2(-b.z, b.x);
        float turned = phi1 - phi0;
        if (turned < -kPi) turned += 2.0f * kPi;
        EXPECT_NEAR(turned, 0.5f, 1.0e-4f) << "the point turned with psi: " << i;
    }
}

TEST(AirflowTipDye, OnlyARotorThatTurnsReleasesAndTheOrderIsTheRotors) {
    const af::BladeShape blade = blade_of(2, 0.1f);
    const af::PolarTable polar = af::estimated_polar();
    const af::LineRotor still = rotor_at({0.2f, 0.5f, 0.2f}, 1.0f, 0.0f, 0.3f, blade, polar);
    const af::LineRotor a = rotor_at({0.4f, 0.5f, 0.2f}, 1.0f, 500.0f, 0.0f, blade, polar);
    const af::LineRotor b = rotor_at({0.6f, 0.5f, 0.2f}, -1.0f, 500.0f, 0.0f, blade, polar);
    const std::vector<glm::vec3> pts = af::tip_points(std::vector<af::LineRotor>{still, a, b}, 0.004f, 0.0002f, 2, 1.0f);
    ASSERT_EQ(pts.size(), 8u) << "two turning rotors, 2 sub-azimuths of 2 blades each; the one at rest has none";
    for (std::size_t i = 0; i < 4; ++i) EXPECT_LT(std::fabs(pts[i].x - 0.4f), 0.11f) << "the first rotor's first: " << i;
    for (std::size_t i = 4; i < 8; ++i) EXPECT_LT(std::fabs(pts[i].x - 0.6f), 0.11f) << "then the second's: " << i;
}

TEST(AirflowTipDye, APointReleasesAKernelThatSumsToOneAndCentresOnIt) {
    const af::GridShape shape{24, 24, 24, 0.01f};
    std::vector<float> dye(std::size_t{24} * 24 * 24, 0.0f);
    const glm::vec3 point{0.1237f, 0.1111f, 0.1352f};
    const std::vector<glm::vec3> pts{point};
    af::emit_tip_dye(dye, shape, pts, 1.0f);
    double sum = 0.0;
    glm::dvec3 first{0.0};
    for (uint32_t k = 0; k < 24; ++k) {
        for (uint32_t j = 0; j < 24; ++j) {
            for (uint32_t i = 0; i < 24; ++i) {
                const double d = dye[i + 24 * (j + 24 * k)];
                if (d == 0.0) continue;
                sum += d;
                first += d * glm::dvec3((i + 0.5) * 0.01, (j + 0.5) * 0.01, (k + 0.5) * 0.01);
                EXPECT_LT(std::fabs((i + 0.5) * 0.01 - point.x), 0.0201) << "within two cells in x";
                EXPECT_LT(std::fabs((j + 0.5) * 0.01 - point.y), 0.0201) << "and in y";
                EXPECT_LT(std::fabs((k + 0.5) * 0.01 - point.z), 0.0201) << "and in z";
            }
        }
    }
    EXPECT_NEAR(sum, 1.0, 1.0e-5) << "Peskin's kernel is a partition of unity";
    EXPECT_NEAR(first.x / sum, point.x, 1.0e-5);
    EXPECT_NEAR(first.y / sum, point.y, 1.0e-5);
    EXPECT_NEAR(first.z / sum, point.z, 1.0e-5);
}

TEST(AirflowTipDye, TheDyeStaysInZeroToOneAndOnlyRises) {
    const af::GridShape shape{16, 16, 16, 0.01f};
    std::vector<float> dye(std::size_t{16} * 16 * 16, 0.25f);
    const std::vector<glm::vec3> pts{{0.08f, 0.08f, 0.08f}, {0.081f, 0.079f, 0.0805f}, {0.0f, 0.0f, 0.0f}, {-0.2f, 0.5f, 0.1f}};
    std::vector<float> before = dye;
    for (int round = 0; round < 3; ++round) {
        af::emit_tip_dye(dye, shape, pts, 40.0f);
        for (std::size_t i = 0; i < dye.size(); ++i) {
            EXPECT_GE(dye[i], before[i]) << "only rises: " << i;
            EXPECT_LE(dye[i], 1.0f) << "never above 1: " << i;
        }
        before = dye;
    }
    EXPECT_EQ(dye[0 + 16 * (0 + 16 * 15)], 0.25f) << "a cell far from every point is as it was";
    EXPECT_GT(dye[8 + 16 * (8 + 16 * 8)], 0.9f) << "the cell at the points is nearly full";
}

TEST(AirflowTipDye, RefusesWhatItCannotReleaseNamingTheField) {
    const auto refused = [](af::TipRelease r, const char* word) {
        const spade::Result<void> ok = af::check_tip_release(r);
        ASSERT_FALSE(ok.has_value()) << word;
        EXPECT_EQ(ok.error().code, spade::Code::invalid_argument) << word;
        EXPECT_NE(ok.error().context.find(word), std::string::npos) << ok.error().context;
    };
    EXPECT_TRUE(af::check_tip_release(af::TipRelease{}).has_value());
    refused(af::TipRelease{.gain = 0.0f}, "gain");
    refused(af::TipRelease{.gain = std::nanf("")}, "gain");
    refused(af::TipRelease{.tip_frac = 0.0f}, "tip_frac");
    refused(af::TipRelease{.tip_frac = 2.0f}, "tip_frac");
}

}  // namespace
