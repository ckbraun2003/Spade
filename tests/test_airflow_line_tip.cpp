// ---------------------------------------------------------------------------
// test_airflow_line_tip.cpp -- L72k's A1: the line's tip/smearing correction (engine/physics/airflow/line.hpp, THE TIP/SMEARING
// CORRECTION; Meyer Forsting, Pirrung, Ramos-Garcia, Wind Energy Science 4, 369, 2019). CPU, milliseconds.
//   * the segment formula is the Biot-Savart law of a straight filament (against the closed form, in double);
//   * the deviation from the paper (direct quadrature of the helical filaments in place of Pirrung's near-wake model) is TESTED:
//     a straight filament cut into chords reproduces the one-chord formula, and the helix's induction converges with the chords;
//   * the Lamb-Oseen factor f = exp(-x^2 / eps^2): 0 for a core far smaller than the distance, the plain Biot-Savart for a huge one;
//   * the correction's sign (it ADDS downwash at a loaded blade, either spin), its blade-by-blade symmetry, its fixed point.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <vector>

#include "physics/airflow/blade.hpp"
#include "physics/airflow/line.hpp"

namespace {

namespace af = spade::physics::airflow;

constexpr double kPi = std::numbers::pi;

// The closed form of a straight filament from a to b (double): Gamma / (4 pi d) (cos t1 - cos t2) about the axis' normal direction.
glm::dvec3 exact_segment(glm::dvec3 p, glm::dvec3 a, glm::dvec3 b) {
    const glm::dvec3 l = b - a;
    const glm::dvec3 r1 = p - a;
    const glm::dvec3 r2 = p - b;
    const glm::dvec3 c = glm::cross(r1, r2);
    const double c2 = glm::dot(c, c);
    return c / c2 * glm::dot(l, r1 / glm::length(r1) - r2 / glm::length(r2)) / (4.0 * kPi);
}

glm::dvec3 to_d(glm::vec3 v) { return {v.x, v.y, v.z}; }

af::BladeShape test_blade(uint32_t blades) {
    af::BladeShape b;
    b.blades = blades;
    b.radius = 0.12f;
    b.stations = {{0.02f, 0.02f, 25.0f}, {0.06f, 0.018f, 15.0f}, {0.12f, 0.010f, 8.0f}};
    return b;
}

// One blade's elements at azimuth 0, ref +x, axis +y, spin as asked.
struct Rig {
    af::BladeShape blade;
    af::PolarTable polar = af::estimated_polar();
    af::LineRotor rotor;
    std::vector<af::LineElement> els;
    float dx = 0.004f;
    explicit Rig(float spin, uint32_t blades = 2, float omega = 500.0f) : blade(test_blade(blades)) {
        rotor.centre = {0.5f, 0.5f, 0.5f};
        rotor.spin = spin;
        rotor.omega = omega;
        rotor.blade = &blade;
        rotor.polar = &polar;
        els = af::line_elements(rotor, dx, 1.0e-4f, 1);
    }
};

}  // namespace

TEST(AirflowLineTip, SegmentIsTheBiotSavartLawOfAStraightFilament) {
    const glm::vec3 a(-0.3f, 0.0f, 0.0f);
    const glm::vec3 b(0.7f, 0.0f, 0.0f);  // not symmetric about the foot of the perpendicular
    const glm::vec3 p(0.1f, 0.02f, 0.01f);
    const glm::dvec3 want = exact_segment(to_d(p), to_d(a), to_d(b));
    const glm::vec3 got = af::line_segment_induction(p, a, b, 0.0f);
    EXPECT_NEAR(got.x, want.x, 1.0e-5 * glm::length(want));
    EXPECT_NEAR(got.y, want.y, 1.0e-5 * glm::length(want));
    EXPECT_NEAR(got.z, want.z, 1.0e-5 * glm::length(want));
    // A long filament is the infinite line's Gamma / (2 pi d), by the right-hand rule: x cross y is +z.
    const glm::vec3 inf = af::line_segment_induction({0.0f, 0.05f, 0.0f}, {-1000.0f, 0.0f, 0.0f}, {1000.0f, 0.0f, 0.0f}, 0.0f);
    EXPECT_NEAR(inf.z, 1.0f / (2.0f * static_cast<float>(kPi) * 0.05f), 1.0e-3f);
    EXPECT_NEAR(inf.x, 0.0f, 1.0e-4f);
    EXPECT_NEAR(inf.y, 0.0f, 1.0e-4f);
}

TEST(AirflowLineTip, AStraightFilamentCutIntoChordsIsTheOneChordFormula) {
    // The deviation from the paper is the quadrature: chords of a polyline. Cut a straight filament 12 ways and 1 way: the same.
    const glm::vec3 a(0.0f, 0.0f, 0.0f);
    const glm::vec3 b(0.0f, 0.0f, 0.2f);
    const glm::vec3 p(0.03f, 0.01f, 0.05f);
    const glm::vec3 whole = af::line_segment_induction(p, a, b, 0.0f);
    glm::vec3 sum(0.0f);
    for (int j = 0; j < 12; ++j) {
        sum += af::line_segment_induction(p, a + (b - a) * (static_cast<float>(j) / 12.0f),
                                          a + (b - a) * (static_cast<float>(j + 1) / 12.0f), 0.0f);
    }
    EXPECT_LT(glm::length(sum - whole), 2.0e-5f * glm::length(whole));
}

TEST(AirflowLineTip, TheCoreFactorIsLambOseensMissingShare) {
    const glm::vec3 a(-1.0f, 0.0f, 0.0f);
    const glm::vec3 b(1.0f, 0.0f, 0.0f);
    const glm::vec3 p(0.0f, 0.04f, 0.0f);  // distance 0.04
    const glm::vec3 plain = af::line_segment_induction(p, a, b, 0.0f);
    // f = exp(-(d / eps)^2): eps = d gives exp(-1); a huge core gives the plain law; a tiny one nothing.
    const glm::vec3 one = af::line_segment_induction(p, a, b, 0.04f);
    EXPECT_NEAR(one.z, plain.z * std::exp(-1.0f), 1.0e-4f * plain.z);
    const glm::vec3 big = af::line_segment_induction(p, a, b, 100.0f);
    EXPECT_NEAR(big.z, plain.z, 1.0e-4f * plain.z);
    const glm::vec3 small = af::line_segment_induction(p, a, b, 0.002f);
    EXPECT_LT(std::fabs(small.z), 1.0e-6f * plain.z);
}

TEST(AirflowLineTip, TheHelixInductionConvergesWithTheChords) {
    // The tip element of a loaded blade, the air as the sampler would give it (a downwash of 4 m/s everywhere); the chords 12 (the
    // default) against 192 (graded chords). The stated tolerance: the correction at 12 chords within 1% of the one at 96.
    Rig rig(1.0f);
    const std::size_t n = rig.els.size();
    std::vector<glm::vec3> air12(n, glm::vec3(0.0f, -4.0f, 0.0f));
    std::vector<glm::vec3> air96 = air12;
    af::LineTipParams p12;
    af::LineTipParams p96;
    p96.segments = 192;
    (void)af::line_tip_correct(rig.rotor, rig.els, rig.dx, 1.225f, air12, p12);
    (void)af::line_tip_correct(rig.rotor, rig.els, rig.dx, 1.225f, air96, p96);
    float worst = 0.0f;
    float biggest = 0.0f;
    for (std::size_t e = 0; e < n; ++e) {
        const glm::vec3 d12 = air12[e] - glm::vec3(0.0f, -4.0f, 0.0f);
        const glm::vec3 d96 = air96[e] - glm::vec3(0.0f, -4.0f, 0.0f);
        const float err = glm::length(d12 - d96);
        worst = std::max(worst, err);
        biggest = std::max(biggest, glm::length(d96));
        // The stated tolerance: within 3% of the fine correction, or 0.02 m/s (0.5% of the 4 m/s sampled downwash) where it is small.
        EXPECT_LT(err, std::max(0.03f * glm::length(d96), 0.02f)) << "element " << e;
    }
    std::printf("tip correction, 12 chords against 192: worst difference %.4f m/s, largest correction %.4f m/s\n", worst, biggest);
}

TEST(AirflowLineTip, ItAddsDownwashAtALoadedBladeWhicheverTheSpin) {
    // Down is -axis (the rotor pushes the air along -axis: ua = -(u . axis) > 0). The correction, from a still sampled air that
    // gives every element a load, must increase ua, most at the tip.
    for (const float spin : {1.0f, -1.0f}) {
        Rig rig(spin);
        std::vector<glm::vec3> air(rig.els.size(), glm::vec3(0.0f));
        const std::vector<glm::vec3> still = air;
        (void)af::line_tip_correct(rig.rotor, rig.els, rig.dx, 1.225f, air);
        const std::size_t per = rig.els.size() / 2;  // 2 blades, 1 sub-azimuth
        double ua_tip = 0.0;
        std::printf("tip correction, spin %+.0f, still air: ua (m/s) by element hub to tip:", static_cast<double>(spin));
        for (std::size_t e = 0; e < per; ++e) std::printf(" %.3f", -static_cast<double>(air[e].y));
        std::printf("\n");
        // The tip's last fifth of the span (where the smeared field misses the most), every element: more air through the rotor.
        for (std::size_t e = per - per / 5; e < per; ++e) {
            ua_tip = -static_cast<double>(air[e].y);
            EXPECT_GT(ua_tip, 0.0) << "spin " << spin << " element " << e;
        }
        EXPECT_GT(ua_tip, 0.0) << "spin " << spin;
    }
}

TEST(AirflowLineTip, TheBladesOfARotorAreCorrectedAlike) {
    // Two blades 180 degrees apart carry the same load: their elements' corrections are the same up to the rotation.
    Rig rig(1.0f, 2);
    std::vector<glm::vec3> air(rig.els.size(), glm::vec3(0.0f, -3.0f, 0.0f));
    (void)af::line_tip_correct(rig.rotor, rig.els, rig.dx, 1.225f, air);
    const std::size_t per = rig.els.size() / 2;
    for (std::size_t e = 0; e < per; ++e) {
        // (float rounding of the 180 degree rotation, amplified by the near field: 0.5% of the value or 1e-4 m/s)
        EXPECT_NEAR(air[e].y, air[per + e].y, 5.0e-3f * std::fabs(air[e].y) + 1.0e-4f) << e;
        EXPECT_NEAR(air[e].x, -air[per + e].x, 5.0e-3f * std::fabs(air[e].x) + 1.0e-4f) << e;
        EXPECT_NEAR(air[e].z, -air[per + e].z, 5.0e-3f * std::fabs(air[e].z) + 1.0e-4f) << e;
    }
}

TEST(AirflowLineTip, TheFixedPointSettles) {
    // The correction depends on the loads, the loads on the corrected air: the iteration's last change is small.
    Rig rig(1.0f);
    std::vector<glm::vec3> air(rig.els.size(), glm::vec3(0.0f, -3.0f, 0.0f));
    af::LineTipParams p;
    p.iterations = 12;
    for (int n = 1; n <= 12; ++n) {
        std::vector<glm::vec3> trial(rig.els.size(), glm::vec3(0.0f, -3.0f, 0.0f));
        af::LineTipParams q;
        q.iterations = n;
        const float change = af::line_tip_correct(rig.rotor, rig.els, rig.dx, 1.225f, trial, q);
        std::printf("tip correction, %d passes: last change %.4f m/s, tip ua %.3f\n", n, static_cast<double>(change),
                    -static_cast<double>(trial[rig.els.size() / 2 - 1].y));
    }
    const float last = af::line_tip_correct(rig.rotor, rig.els, rig.dx, 1.225f, air, p);
    EXPECT_LT(last, 0.02f);  // m/s after 12 passes
    // The default's passes against 12: the corrected air within 0.1 m/s (a few percent of a typical 3 m/s inflow).
    std::vector<glm::vec3> a_def(rig.els.size(), glm::vec3(0.0f, -3.0f, 0.0f));
    (void)af::line_tip_correct(rig.rotor, rig.els, rig.dx, 1.225f, a_def);
    float worst = 0.0f;
    for (std::size_t e = 0; e < air.size(); ++e) worst = std::max(worst, glm::length(a_def[e] - air[e]));
    std::printf("tip correction, default passes against 12: worst difference %.4f m/s; ua by element, 3 m/s sampled:", static_cast<double>(worst));
    for (std::size_t e = 0; e < air.size() / 2; ++e) std::printf(" %.3f", -static_cast<double>(a_def[e].y));
    std::printf("\n");
    EXPECT_LT(worst, 0.1f);
}

TEST(AirflowLineTip, NoCorrectionWhereTheRotorDoesNotTurnOrNothingIsLoaded) {
    Rig rig(1.0f, 2, 0.0f);  // omega 0: no wake geometry
    std::vector<glm::vec3> air(rig.els.size(), glm::vec3(0.1f, -0.2f, 0.3f));
    const std::vector<glm::vec3> before = air;
    (void)af::line_tip_correct(rig.rotor, rig.els, rig.dx, 1.225f, air);
    for (std::size_t e = 0; e < air.size(); ++e) EXPECT_EQ(air[e], before[e]);
}

// The flip (L72k): the correction is ON by default, so a rotor made without saying so has it, and a base-line pin says "off" itself.
TEST(AirflowLineTip, TheCorrectionIsOnByDefaultSinceTheFlip) {
    const af::LineRotor rotor;
    EXPECT_TRUE(rotor.tip_correction);
    af::LineRotor off;
    off.tip_correction = false;
    EXPECT_FALSE(off.tip_correction) << "and off is still there, bit for bit the line before A1";
}
