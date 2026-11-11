// ---------------------------------------------------------------------------
// test_airflow_blade.cpp -- the blade's section load and the grid-free BEMT reference (P4a, L61;
// engine/physics/airflow/blade.hpp, the design note .superpowers/sdd/2026-11-10-wake/p4-actuator-line-design.md).
//
//   AirflowBlade  the diamond angle is monotone around the circle; a polar is refused by name when malformed; the estimated
//                 polar follows its stated formula at its nodes; one section's load by hand (a pure-lift case, a 45 degree
//                 inflow with both lift and drag, a still section); a blade refused by name; the BEMT reference against the
//                 small-angle closed form (Leishman: lambda(r) = (sigma a / 16) (sqrt(1 + 32 theta r / (sigma a)) - 1)) and
//                 its refusals.
//
// Mutations (P4a): the drag's sine and cosine swapped (the 45 degree case); the inflow angle taken the wrong way round (alpha
// = beta + phi: the 45 degree case); the momentum side without the 4 pi (the closed form); the stall blend dropped (the
// estimated polar's nodes).
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include "bemt_reference.hpp"
#include "core/fp32_math.hpp"
#include "physics/airflow/blade.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace math = spade::math;
namespace bt = spade::testing;

constexpr float kDeg = std::numbers::pi_v<float> / 180.0f;

float sin_deg(float d) { return math::sin32(d * kDeg); }
float cos_deg(float d) { return math::cos32(d * kDeg); }

af::PolarTable table_of(const std::vector<float>& alpha, const std::vector<float>& cl, const std::vector<float>& cd) {
    auto p = af::make_polar(alpha, cl, cd);
    EXPECT_TRUE(p.has_value()) << (p ? "" : p.error().context);
    return p ? *p : af::PolarTable{};
}

TEST(AirflowBlade, TheDiamondAngleIsMonotoneAroundTheCircle) {
    float last = -3.0f;
    for (int d = -179; d <= 179; ++d) {
        const float k = af::diamond_angle(cos_deg(static_cast<float>(d)), sin_deg(static_cast<float>(d)));
        EXPECT_GT(k, last) << "at " << d << " degrees";
        last = k;
    }
    EXPECT_NEAR(af::diamond_angle(1.0f, 0.0f), 0.0f, 1e-7f);
    EXPECT_NEAR(af::diamond_angle(0.0f, 1.0f), 1.0f, 1e-6f);
    EXPECT_NEAR(af::diamond_angle(0.0f, -1.0f), -1.0f, 1e-6f);
    EXPECT_NEAR(af::diamond_angle(-1.0f, 0.0f), 2.0f, 1e-6f);
}

TEST(AirflowBlade, AMalformedPolarIsRefusedByName) {
    const std::vector<float> a{-10.0f, 0.0f, 10.0f};
    const std::vector<float> v{0.0f, 0.1f, 0.2f};
    EXPECT_TRUE(af::make_polar(a, v, v).has_value());
    const auto one = af::make_polar(std::vector<float>{0.0f}, std::vector<float>{0.0f}, std::vector<float>{0.0f});
    ASSERT_FALSE(one.has_value());
    EXPECT_NE(one.error().context.find("at least 2 points"), std::string::npos) << one.error().context;
    const auto lengths = af::make_polar(a, v, std::vector<float>{0.0f, 0.1f});
    ASSERT_FALSE(lengths.has_value());
    EXPECT_NE(lengths.error().context.find("length"), std::string::npos) << lengths.error().context;
    const auto order = af::make_polar(std::vector<float>{-10.0f, 10.0f, 0.0f}, v, v);
    ASSERT_FALSE(order.has_value());
    EXPECT_NE(order.error().context.find("alpha_deg[2]"), std::string::npos) << order.error().context;
    const auto bad = af::make_polar(a, std::vector<float>{0.0f, std::nanf(""), 0.2f}, v);
    ASSERT_FALSE(bad.has_value());
    EXPECT_NE(bad.error().context.find("finite"), std::string::npos) << bad.error().context;
    const auto range = af::make_polar(std::vector<float>{-10.0f, 0.0f, 190.0f}, v, v);
    ASSERT_FALSE(range.has_value());
    EXPECT_NE(range.error().context.find("[-180, 180]"), std::string::npos) << range.error().context;
}

// The estimated polar (blade.hpp): attached C_l = 5.5 (alpha + 3 deg) in radians for alpha in [-17, 11] degrees, blended
// linearly in alpha to the flat plate 0.9 sin 2 alpha over 14 degrees more on each side, the flat plate beyond; C_d = 0.03
// + 0.02 C_l^2 attached, blended to 0.03 + 1.77 sin^2 alpha the same way.
TEST(AirflowBlade, TheEstimatedPolarFollowsItsFormulaAtItsNodes) {
    const af::PolarTable polar = af::estimated_polar();
    ASSERT_GE(polar.key.size(), 40u);
    const auto flat_cl = [](float d) { return 0.9f * math::sin32(2.0f * d * kDeg); };
    const auto flat_cd = [](float d) {
        const float s = math::sin32(d * kDeg);
        return 0.03f + 1.77f * s * s;
    };
    const auto attached_cl = [](float d) { return 5.5f * (d + 3.0f) * kDeg; };
    const auto attached_cd = [&](float d) {
        const float c = attached_cl(d);
        return 0.03f + 0.02f * c * c;
    };
    const auto expect_at = [&](float d, float cl, float cd) {
        const af::SectionCoefficients s = af::polar_at(polar, cos_deg(d), sin_deg(d));
        EXPECT_NEAR(s.cl, cl, 2e-3f) << "C_l at " << d << " degrees";
        EXPECT_NEAR(s.cd, cd, 2e-3f) << "C_d at " << d << " degrees";
    };
    expect_at(-3.0f, 0.0f, 0.03f);                                  // zero lift
    expect_at(5.0f, attached_cl(5.0f), attached_cd(5.0f));          // attached
    expect_at(11.0f, attached_cl(11.0f), attached_cd(11.0f));       // the last attached degree: C_l 1.34
    expect_at(-17.0f, attached_cl(-17.0f), attached_cd(-17.0f));    // and its mirror about -3
    const float stall_cl = attached_cl(11.0f);
    const float stall_cd = attached_cd(11.0f);
    expect_at(18.0f, stall_cl + 0.5f * (flat_cl(25.0f) - stall_cl), stall_cd + 0.5f * (flat_cd(25.0f) - stall_cd));  // mid-blend
    expect_at(25.0f, flat_cl(25.0f), flat_cd(25.0f));  // the flat plate from here
    expect_at(90.0f, 0.0f, 1.8f);
    expect_at(-60.0f, flat_cl(-60.0f), flat_cd(-60.0f));
    expect_at(150.0f, flat_cl(150.0f), flat_cd(150.0f));
}

// A polar read only at its nodes: (-20, 0, 10, 20) degrees.
TEST(AirflowBlade, OneSectionsLoadByHand) {
    const af::PolarTable polar = table_of({-20.0f, 0.0f, 10.0f, 20.0f}, {-0.8f, 0.1f, 0.9f, 1.0f}, {0.05f, 0.02f, 0.04f, 0.08f});
    const float rho = 1.2f;
    {  // no inflow: phi = 0, alpha = beta = 10 degrees: lift along the thrust, drag against the motion.
        const af::SectionLoad l = af::section_load(polar, 0.01f, sin_deg(10.0f), cos_deg(10.0f), 0.0f, 40.0f, rho);
        const float q = 0.5f * rho * 1600.0f * 0.01f;  // 9.6 N/m
        EXPECT_NEAR(l.axial, q * 0.9f, 1e-4f * q);
        EXPECT_NEAR(l.tangent, q * 0.04f, 1e-4f * q);
    }
    {  // ua = wt = 10 m/s: phi = 45 degrees; beta 45 degrees: alpha = 0, C_l 0.1, C_d 0.02, both lift and drag resolved.
        const af::SectionLoad l = af::section_load(polar, 0.02f, sin_deg(45.0f), cos_deg(45.0f), 10.0f, 10.0f, rho);
        const float q = 0.5f * rho * 200.0f * 0.02f;  // 2.4 N/m
        const float lift = q * 0.1f;
        const float drag = q * 0.02f;
        const float r2 = 0.70710678f;
        EXPECT_NEAR(l.axial, (lift - drag) * r2, 1e-4f * q);
        EXPECT_NEAR(l.tangent, (lift + drag) * r2, 1e-4f * q);
    }
    {  // nothing moving: nothing.
        const af::SectionLoad l = af::section_load(polar, 0.01f, 0.1f, 0.99f, 0.0f, 0.0f, rho);
        EXPECT_EQ(l.axial, 0.0f);
        EXPECT_EQ(l.tangent, 0.0f);
    }
}

TEST(AirflowBlade, ABladeIsRefusedByName) {
    af::BladeShape ok;
    ok.blades = 2;
    ok.radius = 0.1f;
    ok.stations = {{0.01f, 0.01f, 10.0f}, {0.1f, 0.01f, 10.0f}};
    EXPECT_TRUE(af::check_blade(ok).has_value());
    const auto refused = [&](af::BladeShape b, const char* field) {
        const auto r = af::check_blade(b);
        ASSERT_FALSE(r.has_value()) << field;
        EXPECT_NE(r.error().context.find(field), std::string::npos) << r.error().context;
    };
    af::BladeShape b = ok;
    b.blades = 0;
    refused(b, "blades");
    b = ok;
    b.radius = 0.0f;
    refused(b, "radius");
    b = ok;
    b.stations.resize(1);
    refused(b, "stations");
    b = ok;
    b.stations = {{0.05f, 0.01f, 10.0f}, {0.04f, 0.01f, 10.0f}};
    refused(b, "stations[1].r");
    b = ok;
    b.stations = {{0.01f, 0.01f, 10.0f}, {0.12f, 0.01f, 10.0f}};
    refused(b, "stations[1].r");
    b = ok;
    b.stations[0].chord = 0.0f;
    refused(b, "stations[0].chord");
    b = ok;
    b.stations[1].beta_deg = std::nanf("");
    refused(b, "stations[1].beta_deg");
}

// Constant chord and pitch, a linear lift slope and no drag: the momentum balance of each annulus, 4 lambda^2 r = (sigma a / 2)
// (theta r^2 - lambda r) in the small angle, has Leishman's closed form for lambda(r) = v_i / (omega R), so C_T is an integral
// of it. The BEMT keeps the true sines and cosines, so it differs by the small-angle error: below 5% here (phi up to 0.3
// near the hub, 0.03 over most of the span).
TEST(AirflowBlade, BemtMatchesTheSmallAngleClosedForm) {
    const float R = 0.1f;
    const float chord = 0.012f;
    const float theta_deg = 6.0f;
    const float a = 5.5f;
    std::vector<float> alpha, cl, cd;
    for (int d = -40; d <= 40; d += 2) {
        alpha.push_back(static_cast<float>(d));
        cl.push_back(a * static_cast<float>(d) * kDeg);
        cd.push_back(0.0f);
    }
    const af::PolarTable polar = table_of(alpha, cl, cd);
    af::BladeShape blade;
    blade.blades = 2;
    blade.radius = R;
    blade.stations = {{0.01f, chord, theta_deg}, {R, chord, theta_deg}};
    ASSERT_TRUE(af::check_blade(blade).has_value());
    const float omega = 400.0f;  // omega R = 40 m/s
    const float rho = 1.2f;
    const auto loads = bt::bemt(bt::BemtInput{.blade = &blade, .polar = &polar, .omega = omega, .rho = rho});
    ASSERT_TRUE(loads.has_value()) << loads.error().context;

    const double sigma = 2.0 * chord / (std::numbers::pi * R);
    const double theta = theta_deg * std::numbers::pi / 180.0;
    double ct = 0.0;
    const int n = 20000;
    const double r0 = 0.1;  // the first station, 0.01 m, is r / R = 0.1
    for (int i = 0; i < n; ++i) {
        const double r = r0 + (1.0 - r0) * (i + 0.5) / n;
        const double lambda = (sigma * a / 16.0) * (std::sqrt(1.0 + 32.0 * theta * r / (sigma * a)) - 1.0);
        ct += (sigma * a / 2.0) * (theta * r * r - lambda * r) * (1.0 - r0) / n;
    }
    const double area = std::numbers::pi * R * R;
    const double thrust = ct * rho * area * (omega * R) * (omega * R);
    EXPECT_GT(loads->thrust, 0.0f);
    EXPECT_NEAR(loads->thrust, thrust, 0.05 * thrust) << "closed form C_T " << ct;
    EXPECT_GT(loads->torque, 0.0f);
    EXPECT_FLOAT_EQ(loads->power, loads->torque * omega);
    EXPECT_GT(loads->inflow_mean, 0.0f);

    // Prandtl's tip loss takes some thrust off and not most of it (V2's comparator).
    const auto lossy = bt::bemt(bt::BemtInput{.blade = &blade, .polar = &polar, .omega = omega, .rho = rho, .tip_loss = true});
    ASSERT_TRUE(lossy.has_value()) << lossy.error().context;
    EXPECT_LT(lossy->thrust, 0.995f * loads->thrust);
    EXPECT_GT(lossy->thrust, 0.80f * loads->thrust);
}

TEST(AirflowBlade, TheBemtRefusesWhatItCannotSolveByName) {
    af::BladeShape blade;
    blade.blades = 2;
    blade.radius = 0.1f;
    blade.stations = {{0.01f, 0.01f, 10.0f}, {0.1f, 0.01f, 10.0f}};
    const af::PolarTable polar = af::estimated_polar();
    const bt::BemtInput good{.blade = &blade, .polar = &polar, .omega = 400.0f};
    EXPECT_TRUE(bt::bemt(good).has_value());
    const auto refused = [&](bt::BemtInput in, const char* word) {
        const auto r = bt::bemt(in);
        ASSERT_FALSE(r.has_value()) << word;
        EXPECT_NE(r.error().context.find(word), std::string::npos) << r.error().context;
    };
    bt::BemtInput in = good;
    in.blade = nullptr;
    refused(in, "blade");
    in = good;
    in.polar = nullptr;
    refused(in, "polar");
    in = good;
    in.omega = -1.0f;
    refused(in, "omega");
    in = good;
    in.rho = 0.0f;
    refused(in, "rho");
    in = good;
    in.climb = -1.0f;
    refused(in, "climb");
    in = good;
    in.annuli = 0;
    refused(in, "annuli");
}

}  // namespace
