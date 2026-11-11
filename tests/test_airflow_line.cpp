// ---------------------------------------------------------------------------
// test_airflow_line.cpp -- the actuator line on the CPU (P4b, L61; engine/physics/airflow/line.hpp, the design note
// .superpowers/sdd/2026-11-10-wake/p4-actuator-line-design.md sections 2 and 6).
//
//   AirflowLine  R1 the elements: P11's convention (blade b at s psi + 2 pi b / B about the axis, both hands), the count and spacing, the
//                chord and pitch interpolated, the sub-azimuths; R5 the kernel's sample: a uniform flow returned exactly, a linear one at its
//                point; R3 one rotor's load in still air against section_load() summed by hand, and its signs; R4 the spread conserves the
//                impulse and the angular momentum about the axis; the sub-azimuth spread conserves the same and differs in pattern; a
//                still rotor adds nothing; R7 every refusal by name.
//
// Mutations (P4b): the tangent force on the air with the wrong sign (the torque test); the kernel's sum not normalised (the uniform
// sample and the conservation); the spin ignored in the azimuth (the left hand); the elements' span not dr (the load by hand).
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "core/fp32_math.hpp"
#include "physics/airflow/blade.hpp"
#include "physics/airflow/line.hpp"

namespace {

namespace af = spade::physics::airflow;
namespace math = spade::math;

constexpr float kDx = 0.004f;
constexpr float kRho = 1.225f;
constexpr float kTwoPi = 2.0f * std::numbers::pi_v<float>;

af::BladeShape blade_of(uint32_t blades = 2) {
    af::BladeShape b;
    b.blades = blades;
    b.radius = 0.0762f;
    b.stations = {{0.0114f, 0.0085f, 30.0f}, {0.0457f, 0.0148f, 22.0f}, {0.0762f, 0.0010f, 14.0f}};
    return b;
}

// A rotor in the middle of a 64-cell grid at 4 mm: 0.256 m, reach R + 6 dx = 0.100 m.
af::LineRotor rotor_of(const af::BladeShape& blade, const af::PolarTable& polar) {
    af::LineRotor r;
    r.centre = glm::vec3(0.128f);
    r.blade = &blade;
    r.polar = &polar;
    r.omega = 400.0f;
    return r;
}

af::MacGrid grid_64() {
    af::MacGrid g;
    g.shape = {64, 64, 64, kDx};
    g.u.assign(65u * 64u * 64u, 0.0f);
    g.v.assign(64u * 65u * 64u, 0.0f);
    g.w.assign(64u * 64u * 65u, 0.0f);
    g.p.assign(64u * 64u * 64u, 0.0f);
    return g;
}

struct Increments {
    std::vector<float> du, dv, dw;
    explicit Increments(const af::MacGrid& g) : du(g.u.size(), 0.0f), dv(g.v.size(), 0.0f), dw(g.w.size(), 0.0f) {}
};

TEST(AirflowLine, TheElementsFollowP11sConventionInBothHands) {
    const af::BladeShape blade = blade_of(2);
    const af::PolarTable polar = af::estimated_polar();
    af::LineRotor r = rotor_of(blade, polar);
    r.psi = 0.3f;
    const std::vector<af::LineElement> el = af::line_elements(r, kDx, 0.0f, 1);
    const uint32_t per_blade = static_cast<uint32_t>(std::ceil((0.0762f - 0.0114f) / kDx));  // 17
    ASSERT_EQ(el.size(), 2u * per_blade);
    for (uint32_t b = 0; b < 2; ++b) {
        const float phi = 0.3f + static_cast<float>(b) * std::numbers::pi_v<float>;
        for (uint32_t e = 0; e < per_blade; ++e) {
            const af::LineElement& x = el[b * per_blade + e];
            const glm::vec3 d = x.pos - r.centre;
            EXPECT_NEAR(glm::length(d), x.r, 1e-6f);
            EXPECT_NEAR(d.x, x.r * math::cos32(phi), 1e-5f) << "blade " << b << " element " << e;
            EXPECT_NEAR(d.y, 0.0f, 1e-6f);
            EXPECT_NEAR(d.z, -x.r * math::sin32(phi), 1e-5f);  // axis x ref = Y x X = -Z
            // It moves along s (axis x d) = d(d)/d(phi) for the right hand: Y x (cos, 0, -sin) = (-sin, 0, -cos).
            EXPECT_NEAR(glm::length(x.motion), 1.0f, 1e-6f);
            EXPECT_NEAR(glm::dot(x.motion, d), 0.0f, 1e-5f);
            EXPECT_NEAR(x.motion.y, 0.0f, 1e-6f);
            EXPECT_NEAR(x.motion.x, -math::sin32(phi), 1e-5f);
            EXPECT_NEAR(x.motion.z, -math::cos32(phi), 1e-5f);
        }
    }
    // The span: equal dr, midpoints between the first station and the tip.
    const float dr = (0.0762f - 0.0114f) / static_cast<float>(per_blade);
    EXPECT_NEAR(el[0].dr, dr, 1e-7f);
    EXPECT_NEAR(el[0].r, 0.0114f + 0.5f * dr, 1e-6f);
    EXPECT_NEAR(el[per_blade - 1].r, 0.0762f - 0.5f * dr, 1e-6f);
    // Chord and pitch are interpolated between stations: the first element is on the first span (0.0114 to 0.0457).
    const float t0 = (el[0].r - 0.0114f) / (0.0457f - 0.0114f);
    EXPECT_NEAR(el[0].chord, 0.0085f + t0 * (0.0148f - 0.0085f), 1e-6f);
    const float beta0 = (30.0f + t0 * (22.0f - 30.0f)) * std::numbers::pi_v<float> / 180.0f;
    EXPECT_NEAR(el[0].sin_beta, math::sin32(beta0), 1e-5f);
    EXPECT_NEAR(el[0].cos_beta, math::cos32(beta0), 1e-5f);

    // The left hand: the azimuth runs the other way, phi = -psi, and the blade moves along -(...).
    r.spin = -1.0f;
    const std::vector<af::LineElement> left = af::line_elements(r, kDx, 0.0f, 1);
    ASSERT_EQ(left.size(), el.size());
    const glm::vec3 dl = left[0].pos - r.centre;
    EXPECT_NEAR(dl.x, left[0].r * math::cos32(0.3f), 1e-5f);
    EXPECT_NEAR(dl.z, left[0].r * math::sin32(0.3f), 1e-5f);  // phi = -0.3: -r sin(-0.3) = +r sin 0.3
    EXPECT_NEAR(left[0].motion.x, -math::sin32(0.3f), 1e-5f);  // s (axis x d), s = -1, d = (cos 0.3, 0, sin 0.3)
    EXPECT_NEAR(left[0].motion.z, math::cos32(0.3f), 1e-5f);
    EXPECT_NEAR(glm::length(left[0].motion), 1.0f, 1e-6f);
    EXPECT_NEAR(glm::dot(left[0].motion, dl), 0.0f, 1e-5f);

    // Sub-azimuths: n_sub 4 gives 4 sets, set k at psi + omega h (k + 1/2) / 4.
    r.spin = 1.0f;
    const float h = 0.0002f;
    const std::vector<af::LineElement> sub = af::line_elements(r, kDx, h, 4);
    ASSERT_EQ(sub.size(), 4u * el.size());
    const float phi2 = 0.3f + 400.0f * h * 2.5f / 4.0f;
    const glm::vec3 d2 = sub[2u * el.size()].pos - r.centre;
    EXPECT_NEAR(d2.x, sub[2u * el.size()].r * math::cos32(phi2), 1e-5f);
    EXPECT_NEAR(d2.z, -sub[2u * el.size()].r * math::sin32(phi2), 1e-5f);
}

TEST(AirflowLine, TheKernelSamplesAUniformFlowExactlyAndALinearOneAtItsPoint) {
    af::MacGrid g = grid_64();
    std::fill(g.u.begin(), g.u.end(), 1.5f);
    std::fill(g.v.begin(), g.v.end(), -0.7f);
    std::fill(g.w.begin(), g.w.end(), 0.3f);
    const glm::vec3 at(0.1301f, 0.1127f, 0.1442f);
    const glm::vec3 s = af::line_sample(g, at);
    // To float rounding of a sum of ~2000 weighted terms (13 ulp seen), not bit-exact.
    EXPECT_NEAR(s.x, 1.5f, 1e-5f);
    EXPECT_NEAR(s.y, -0.7f, 1e-5f);
    EXPECT_NEAR(s.z, 0.3f, 1e-5f);

    // u = a x on the u-faces (x = i dx), v = b y, w = c z: the mean at the point is the field at the point.
    const float a = 0.8f, b = -1.1f, c = 0.5f;
    for (uint32_t k = 0; k < 64; ++k) {
        for (uint32_t j = 0; j < 64; ++j) {
            for (uint32_t i = 0; i <= 64; ++i) g.u[af::u_index(g.shape, i, j, k)] = a * static_cast<float>(i) * kDx;
        }
    }
    for (uint32_t k = 0; k < 64; ++k) {
        for (uint32_t j = 0; j <= 64; ++j) {
            for (uint32_t i = 0; i < 64; ++i) g.v[af::v_index(g.shape, i, j, k)] = b * static_cast<float>(j) * kDx;
        }
    }
    for (uint32_t k = 0; k <= 64; ++k) {
        for (uint32_t j = 0; j < 64; ++j) {
            for (uint32_t i = 0; i < 64; ++i) g.w[af::w_index(g.shape, i, j, k)] = c * static_cast<float>(k) * kDx;
        }
    }
    const glm::vec3 l = af::line_sample(g, at);
    EXPECT_NEAR(l.x, a * at.x, 2e-3f * std::fabs(a) * kDx + 1e-6f);
    EXPECT_NEAR(l.y, b * at.y, 2e-3f * std::fabs(b) * kDx + 1e-6f);
    EXPECT_NEAR(l.z, c * at.z, 2e-3f * std::fabs(c) * kDx + 1e-6f);
}

TEST(AirflowLine, OneRotorsLoadInStillAirIsTheSumOfItsSections) {
    const af::BladeShape blade = blade_of(2);
    const af::PolarTable polar = af::estimated_polar();
    const af::LineRotor r = rotor_of(blade, polar);
    const af::MacGrid g = grid_64();
    Increments inc(g);
    const float h = 0.0002f;
    const auto rep = af::spread_line(r, g, kRho, h, 1, inc.du, inc.dv, inc.dw);
    ASSERT_TRUE(rep.has_value()) << rep.error().context;

    // The same sum by hand: still air, ua = 0, wt = omega r.
    double thrust = 0.0;
    double torque = 0.0;
    for (const af::LineElement& e : af::line_elements(r, kDx, h, 1)) {
        const af::SectionLoad l = af::section_load(polar, e.chord, e.sin_beta, e.cos_beta, 0.0f, 400.0f * e.r, kRho);
        thrust += static_cast<double>(l.axial) * e.dr;
        torque += static_cast<double>(l.tangent) * e.r * e.dr;
    }
    EXPECT_GT(thrust, 0.0) << "the blade pitched 14 to 30 degrees in still air lifts";
    EXPECT_GT(torque, 0.0);
    EXPECT_NEAR(rep->thrust, thrust, 1e-4 * thrust);
    EXPECT_NEAR(rep->torque, torque, 1e-4 * torque);
    EXPECT_NEAR(rep->power, torque * 400.0, 1e-4 * torque * 400.0);
}

TEST(AirflowLine, TheSpreadConservesTheImpulseAndTheAngularMomentum) {
    const af::BladeShape blade = blade_of(2);
    const af::PolarTable polar = af::estimated_polar();
    const af::LineRotor r = rotor_of(blade, polar);
    const af::MacGrid g = grid_64();
    const float h = 0.0002f;
    for (const uint32_t n_sub : {1u, 4u}) {
        Increments inc(g);
        const auto rep = af::spread_line(r, g, kRho, h, n_sub, inc.du, inc.dv, inc.dw);
        ASSERT_TRUE(rep.has_value()) << rep.error().context;
        // What the air should take: -axial axis + tangent m per element, over the sub-azimuths' mean.
        glm::dvec3 force(0.0);
        double qz = 0.0;  // the angular momentum about the axis (+Y) of the force: sum of (x - c) x F, y component
        for (const af::LineElement& e : af::line_elements(r, kDx, h, n_sub)) {
            const af::SectionLoad l = af::section_load(polar, e.chord, e.sin_beta, e.cos_beta, 0.0f, 400.0f * e.r, kRho);
            const glm::dvec3 f = (glm::dvec3(0.0, -1.0, 0.0) * static_cast<double>(l.axial) +
                                  glm::dvec3(e.motion) * static_cast<double>(l.tangent)) *
                                 (static_cast<double>(e.dr) / n_sub);
            force += f;
            const glm::dvec3 arm = glm::dvec3(e.pos - r.centre);
            qz += arm.z * f.x - arm.x * f.z;
        }
        glm::dvec3 got(0.0);
        const double cell = static_cast<double>(kRho) * kDx * kDx * kDx;
        for (const float v : inc.du) got.x += static_cast<double>(v);
        for (const float v : inc.dv) got.y += static_cast<double>(v);
        for (const float v : inc.dw) got.z += static_cast<double>(v);
        got *= cell / h;
        const double scale = std::fabs(force.y);
        EXPECT_NEAR(got.y, force.y, 2e-4 * scale) << "n_sub " << n_sub;
        EXPECT_NEAR(got.x, force.x, 2e-3 * scale);
        EXPECT_NEAR(got.z, force.z, 2e-3 * scale);
        // Angular momentum about the axis: (r x f)_y = z f_x - x f_z summed over the faces, from their own positions.
        double ly = 0.0;
        for (uint32_t k = 0; k < 64; ++k) {
            for (uint32_t j = 0; j < 64; ++j) {
                for (uint32_t i = 0; i <= 64; ++i) {
                    const double z = (static_cast<double>(k) + 0.5) * kDx - r.centre.z;
                    ly += z * static_cast<double>(inc.du[af::u_index(g.shape, i, j, k)]);
                }
            }
        }
        for (uint32_t k = 0; k <= 64; ++k) {
            for (uint32_t j = 0; j < 64; ++j) {
                for (uint32_t i = 0; i < 64; ++i) {
                    const double x = (static_cast<double>(i) + 0.5) * kDx - r.centre.x;
                    ly -= x * static_cast<double>(inc.dw[af::w_index(g.shape, i, j, k)]);
                }
            }
        }
        ly *= cell / h;
        EXPECT_NEAR(ly, qz, 2e-2 * std::fabs(qz)) << "angular momentum about the axis, n_sub " << n_sub;
    }
}

TEST(AirflowLine, FourSubAzimuthsSpreadTheSameThrustInADifferentPattern) {
    const af::BladeShape blade = blade_of(2);
    const af::PolarTable polar = af::estimated_polar();
    const af::LineRotor r = rotor_of(blade, polar);
    const af::MacGrid g = grid_64();
    const float h = 0.0002f;
    Increments one(g);
    Increments four(g);
    const auto a = af::spread_line(r, g, kRho, h, 1, one.du, one.dv, one.dw);
    const auto b = af::spread_line(r, g, kRho, h, 4, four.du, four.dv, four.dw);
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_NEAR(a->thrust, b->thrust, 1e-5 * a->thrust) << "still air: every azimuth has the same load";
    EXPECT_NEAR(a->torque, b->torque, 1e-5 * a->torque);
    EXPECT_NE(one.dw, four.dw) << "the arcs differ";
}

TEST(AirflowLine, AStillRotorInStillAirAddsNothing) {
    const af::BladeShape blade = blade_of(2);
    const af::PolarTable polar = af::estimated_polar();
    af::LineRotor r = rotor_of(blade, polar);
    r.omega = 0.0f;
    const af::MacGrid g = grid_64();
    Increments inc(g);
    const auto rep = af::spread_line(r, g, kRho, 0.0002f, 1, inc.du, inc.dv, inc.dw);
    ASSERT_TRUE(rep.has_value()) << rep.error().context;
    EXPECT_EQ(rep->thrust, 0.0f);
    EXPECT_EQ(rep->torque, 0.0f);
    for (const float v : inc.du) ASSERT_EQ(v, 0.0f);
    for (const float v : inc.dv) ASSERT_EQ(v, 0.0f);
    for (const float v : inc.dw) ASSERT_EQ(v, 0.0f);
}

TEST(AirflowLine, EveryRefusalIsByName) {
    const af::BladeShape blade = blade_of(2);
    const af::PolarTable polar = af::estimated_polar();
    const af::LineRotor good = rotor_of(blade, polar);
    const af::GridShape shape{64, 64, 64, kDx};
    EXPECT_TRUE(af::check_line(good, shape, kRho, 0.0002f, 1).has_value());
    const auto refused = [&](af::LineRotor r, float rho, float h, uint32_t n_sub, const char* word) {
        const auto c = af::check_line(r, shape, rho, h, n_sub);
        ASSERT_FALSE(c.has_value()) << word;
        EXPECT_NE(c.error().context.find(word), std::string::npos) << c.error().context;
    };
    af::LineRotor r = good;
    r.blade = nullptr;
    refused(r, kRho, 0.0002f, 1, "blade");
    r = good;
    r.polar = nullptr;
    refused(r, kRho, 0.0002f, 1, "polar");
    r = good;
    r.omega = -1.0f;
    refused(r, kRho, 0.0002f, 1, "omega");
    r = good;
    r.omega = std::nanf("");
    refused(r, kRho, 0.0002f, 1, "omega");
    r = good;
    r.spin = 0.0f;
    refused(r, kRho, 0.0002f, 1, "spin");
    r = good;
    r.axis = glm::vec3(0.0f, 2.0f, 0.0f);
    refused(r, kRho, 0.0002f, 1, "axis");
    r = good;
    r.ref = glm::vec3(0.0f, 1.0f, 0.0f);
    refused(r, kRho, 0.0002f, 1, "ref");
    r = good;
    r.psi = std::nanf("");
    refused(r, kRho, 0.0002f, 1, "psi");
    r = good;
    r.centre.x = std::nanf("");
    refused(r, kRho, 0.0002f, 1, "centre");
    refused(good, kRho, 0.0002f, 0, "n_sub");
    refused(good, 0.0f, 0.0002f, 1, "rho");
    refused(good, kRho, 0.0f, 1, "h");
    r = good;
    r.centre.x = 0.05f;  // 0.05 - (0.0762 + 0.024) < 0: the kernel leaves the grid
    refused(r, kRho, 0.0002f, 1, "grid");
    // An invalid blade is refused by blade's own check, naming the field.
    af::BladeShape bad = blade;
    bad.blades = 0;
    r = good;
    r.blade = &bad;
    refused(r, kRho, 0.0002f, 1, "blades");
    // And spread_line refuses what check_line refuses, adding nothing.
    const af::MacGrid g = grid_64();
    Increments inc(g);
    af::LineRotor off = good;
    off.centre.x = 0.05f;
    const auto s = af::spread_line(off, g, kRho, 0.0002f, 1, inc.du, inc.dv, inc.dw);
    ASSERT_FALSE(s.has_value());
    for (const float v : inc.dw) ASSERT_EQ(v, 0.0f);
}

}  // namespace
