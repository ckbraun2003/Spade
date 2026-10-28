// ---------------------------------------------------------------------------
// test_airflow_dense_disc.cpp -- the library's actuator disc, dense where the
// grid resolves it: up to 10 rings of 64 points with the swirl (the lead's
// ruling, 2026-10-27), placed as Physics' triage 2 measured it on D1
// (physics/scratch-plan-a-triage 0691b91, ring_points() and scatter()): 3
// rings of 16 drove D1 past 55 m/s from about step 20 (151.5 m/s at step 38);
// 10 rings of 64 stayed under 20.1 m/s over 200 steps, the thrust the same.
// The density follows R/dx (the lead's rule, 2026-10-27), so a coarse grid's
// disc keeps 3 rings of 16 and its speed: D1 (R/dx 19) is 10 x 64, C8 5 x 32,
// the editor's Run air (R/dx 3) 3 x 16.
//
//   AirflowDenseDisc   in the gate: the rule's table, D1's 640 points on the
//                      measured rings, the thrust and torque the increments
//                      carry, the largest single-face increment for D1's
//                      disc, and the run line's note past the cap.
//   AirflowDiscPeak    opt-in (SPADE_DISC_PEAK=1): D1 with its solids through
//                      fluid_step() itself, SPADE_DISC_PEAK_STEPS steps (40),
//                      each step's peak |u, v, w| and its faces over 55 m/s.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <numbers>
#include <vector>

#include <glm/glm.hpp>

#include "b7_harness.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/mac_grid.hpp"

namespace {

using namespace spade::physics::airflow;

// Triage 2's ring_points() with its frame, copied: the measured rule. Ring r
// at R sqrt((r + 0.5) / rings), point m at 2 pi m / angles, its cos and sin
// taken in double and rounded to float.
[[nodiscard]] std::vector<glm::vec3> ring_points(const ActuatorDisc& dc, uint32_t rings, uint32_t angles) {
    const glm::vec3 helper = std::fabs(dc.axis.x) < 0.9f ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    const glm::vec3 e1 = glm::normalize(glm::cross(dc.axis, helper));
    const glm::vec3 e2 = glm::cross(dc.axis, e1);
    std::vector<glm::vec3> pts;
    for (uint32_t ring = 0; ring < rings; ++ring) {
        const float r = dc.radius * std::sqrt((static_cast<float>(ring) + 0.5f) / static_cast<float>(rings));
        for (uint32_t m = 0; m < angles; ++m) {
            const double a = 2.0 * std::numbers::pi * static_cast<double>(m) / static_cast<double>(angles);
            pts.push_back(dc.centre + r * (static_cast<float>(std::cos(a)) * e1 + static_cast<float>(std::sin(a)) * e2));
        }
    }
    return pts;
}

[[nodiscard]] bool same_bits(glm::vec3 a, glm::vec3 b) {
    return std::bit_cast<uint32_t>(a.x) == std::bit_cast<uint32_t>(b.x) &&
           std::bit_cast<uint32_t>(a.y) == std::bit_cast<uint32_t>(b.y) &&
           std::bit_cast<uint32_t>(a.z) == std::bit_cast<uint32_t>(b.z);
}

// The points of `d`'s spread that are not ring_points()' bit for bit.
[[nodiscard]] std::size_t off_the_measured_rings(const ActuatorDisc& d, const DiscSpread& sp) {
    const std::vector<glm::vec3> want = ring_points(d, 10, 64);
    std::size_t off = std::max(want.size(), sp.points.size()) - std::min(want.size(), sp.points.size());
    for (std::size_t i = 0; i < std::min(want.size(), sp.points.size()); ++i) {
        off += same_bits(sp.points[i], want[i]) ? 0u : 1u;
    }
    return off;
}

// D1's air and step (tests/d1_case.hpp; the case's ISA density).
constexpr float kRho = 1.225f;
constexpr float kH = spade::testing::kD1Step;  // 0.38 ms

// D1's disc 0 as triage 2 measured it: the reference quad's rotor at hover,
// m g / 4 with its torque ratio (the air takes -Q for spin +1), radius
// 0.0762 m, at 4 mm; its hub on a cell centre in x and y and on a face in z,
// as in kD1Grid ((147.5, 93.5, 84) cells), here in a 48^3 box that holds its
// reach. `axis` turns it for the rings' check.
[[nodiscard]] ActuatorDisc d1_disc(glm::vec3 axis = {0.0f, -1.0f, 0.0f}) {
    constexpr float kMass = 0.526000023f;
    constexpr float kTorquePerThrust = 2.04075654e-08f / 2.00861859e-06f;  // torque_coeff / thrust_coeff
    const float thrust = kMass * 9.80665f / 4.0f;
    return ActuatorDisc{.centre = {23.5f * 0.004f, 23.5f * 0.004f, 24.0f * 0.004f},
                        .axis = axis,
                        .radius = 0.0762000009f,
                        .thrust = thrust,
                        .torque = -thrust * kTorquePerThrust};
}
constexpr GridShape kBox{48, 48, 48, 0.004f};

struct Increments {
    std::vector<float> du, dv, dw;
};

[[nodiscard]] Increments spread(const ActuatorDisc& d, const GridShape& s) {
    Increments out{std::vector<float>(std::size_t{s.nx + 1u} * s.ny * s.nz, 0.0f),
                   std::vector<float>(std::size_t{s.nx} * (s.ny + 1u) * s.nz, 0.0f),
                   std::vector<float>(std::size_t{s.nx} * s.ny * (s.nz + 1u), 0.0f)};
    spread_actuator(d, s, kRho, kH, out.du, out.dv, out.dw);
    return out;
}

// ===========================================================================
// AirflowDenseDisc
// ===========================================================================

// 640 points on the measured rings, bit for bit: D1's level disc, and a disc
// whose axis takes the frame's other helper. The radii, angles and frame are
// ring_points()'. 10 x 64 is also the cap: the most points any disc takes.
TEST(AirflowDenseDisc, ThePointsAreTenRingsOfSixtyFourAtTheMeasuredRadii) {
    EXPECT_EQ(kDiscRings, 10u) << "the most rings";
    EXPECT_EQ(kDiscAngles, 64u) << "the most angles";
    EXPECT_EQ(kDiscPoints, 640u) << "the most points: the room in DiscSpread and the GPU's record";
    for (const glm::vec3 axis : {glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)}) {
        const ActuatorDisc d = d1_disc(axis);
        ASSERT_TRUE(check_disc(d, kBox).has_value());
        const DiscSpread sp = disc_spread(d, kBox, kRho, kH);
        EXPECT_EQ(sp.count, 640u) << "D1's R/dx 19.05";
        EXPECT_EQ(off_the_measured_rings(d, sp), 0u) << "axis (" << axis.x << ", " << axis.y << ", " << axis.z << ")";
        // The per-point scale: T / 640 and Q / 640 (over each point's r, in its tangent).
        EXPECT_EQ(sp.scale, kH * d.thrust / (640.0f * kRho * kBox.dx * kBox.dx * kBox.dx));
        EXPECT_EQ(sp.swirl_scale, kH * d.torque / (640.0f * kRho * kBox.dx * kBox.dx * kBox.dx));
    }
}

// THE RULE (the lead, 2026-10-27): rings = clamp(ceil(R / 2dx), 3, 10),
// angles the smallest of 16, 32 and 64 at or above pi R/dx (64 if none is),
// computed once in disc_spread() and carried to the GPU's record. At R/dx 2,
// 3, 10, 19 and 25 on a grid whose dx makes R/dx exact, then the library's
// discs at their own R/dx in fp32: the editor's Run air (0.0762 m at 25 mm,
// 3.048), C8 (0.1 at 10 mm, 10.0), C8q (0.0762 at 0.975/128 m, 10.0037: one
// ring more than C8, by the ceil) and D1 (0.0762 at 4 mm, 19.05). Each
// disc's first n points are ring_points()' rings at that density, bit for
// bit, and the slots past n are 0; the per-point scale is T / n.
TEST(AirflowDenseDisc, ThePointsFollowRadiusOverDx) {
    struct Row {
        const char* what;
        float radius, dx;
        uint32_t rings, angles, count;
    };
    const Row rows[] = {
        {"R/dx 2", 2.0f * 0.125f, 0.125f, 3, 16, 48},
        {"R/dx 3", 3.0f * 0.125f, 0.125f, 3, 16, 48},
        {"R/dx 10", 10.0f * 0.125f, 0.125f, 5, 32, 160},
        {"R/dx 19", 19.0f * 0.125f, 0.125f, 10, 64, 640},
        {"R/dx 25", 25.0f * 0.125f, 0.125f, 10, 64, 640},
        {"Run air 32^3", 0.0762f, 0.025f, 3, 16, 48},
        {"C8", 0.1f, 0.01f, 5, 32, 160},
        {"C8q", 0.0762f, 0.975f / 128.0f, 6, 32, 192},
        {"D1", 0.0762f, 0.004f, 10, 64, 640},
    };
    for (const Row& row : rows) {
        // A box that holds the disc's reach, its centre off the faces' grid lines.
        const uint32_t n = static_cast<uint32_t>(2.0f * (row.radius / row.dx)) + 8u;
        const GridShape shape{n, n, n, row.dx};
        const float mid = 0.5f * static_cast<float>(n) * row.dx + 0.25f * row.dx;
        const ActuatorDisc d{.centre = {mid, mid, mid}, .axis = {0.0f, -1.0f, 0.0f}, .radius = row.radius,
                             .thrust = 1.0f, .torque = -0.01f};
        ASSERT_TRUE(check_disc(d, shape).has_value()) << row.what;
        const DiscSpread sp = disc_spread(d, shape, kRho, kH);
        EXPECT_EQ(sp.rings, row.rings) << row.what;
        EXPECT_EQ(sp.angles, row.angles) << row.what;
        EXPECT_EQ(sp.count, row.count) << row.what;
        const std::vector<glm::vec3> want = ring_points(d, row.rings, row.angles);
        std::size_t off = 0;
        for (std::size_t i = 0; i < want.size(); ++i) off += same_bits(sp.points[i], want[i]) ? 0u : 1u;
        EXPECT_EQ(off, 0u) << row.what << ": points off ring_points()' rings";
        std::size_t stray = 0;
        for (std::size_t i = row.count; i < kDiscPoints; ++i) {
            stray += sp.points[i] == glm::vec3(0.0f) && sp.tangent[i] == glm::vec3(0.0f) ? 0u : 1u;
        }
        EXPECT_EQ(stray, 0u) << row.what << ": slots past n not 0";
        EXPECT_EQ(sp.scale, kH * d.thrust / (static_cast<float>(row.count) * kRho * row.dx * row.dx * row.dx))
            << row.what;
    }
}

// Past the cap (R/dx 20) the points spread over 2 dx apart: the run line
// (LiveRun::run_label()) says so, naming the discs and their spacing in dx,
// the larger of the outer ring's arc (2 pi r_outer / angles) and the rings'
// pitch (R / rings). At or under the cap it says nothing: D1 is 1.91 dx.
TEST(AirflowDenseDisc, TheRunLineNotesDiscsWhosePointsAreOverTwoDxApart) {
    constexpr float dx = 0.125f;
    EXPECT_EQ(disc_spacing_note(std::vector<float>{}, dx), "");
    EXPECT_EQ(disc_spacing_note(std::vector<float>{3.0f * dx, 19.0f * dx, 20.0f * dx}, dx), "")
        << "the cap and under it: 2 dx at most";
    EXPECT_EQ(disc_spacing_note(std::vector<float>(4, 0.0762f), 0.004f), "") << "D1";
    EXPECT_EQ(disc_spacing_note(std::vector<float>{25.0f * dx}, dx),
              "disc 0 at R/dx 25.0 spaces its points 2.5 dx apart (over 2 dx)");
    EXPECT_EQ(disc_spacing_note(std::vector<float>(4, 25.0f * dx), dx),
              "discs 0-3 at R/dx 25.0 space their points 2.5 dx apart (over 2 dx)");
    EXPECT_EQ(disc_spacing_note(std::vector<float>{19.0f * dx, 25.0f * dx, 30.0f * dx, 25.0f * dx}, dx),
              "discs 1, 3 at R/dx 25.0 space their points 2.5 dx apart (over 2 dx); "
              "disc 2 at R/dx 30.0 spaces its points 3.0 dx apart (over 2 dx)");
}

// The increments carry the disc's thrust impulse along its axis and its
// torque's about it, as 3 rings of 16 did: D1's -5.15830 N is 4 of these.
TEST(AirflowDenseDisc, TheIncrementsCarryTheThrustAndTheTorque) {
    const ActuatorDisc d = d1_disc();
    const Increments inc = spread(d, kBox);
    const double cell = static_cast<double>(kBox.dx) * kBox.dx * kBox.dx;
    const std::vector<float>* comp[3] = {&inc.du, &inc.dv, &inc.dw};
    glm::dvec3 momentum(0.0);
    glm::dvec3 angular(0.0);
    for (uint32_t c = 0; c < 3; ++c) {
        for_each_sample(kBox, c, [&](std::size_t idx, glm::vec3 x) {
            glm::dvec3 f(0.0);
            f[static_cast<glm::length_t>(c)] = static_cast<double>((*comp[c])[idx]) * kRho * cell;
            momentum += f;
            angular += glm::cross(glm::dvec3(x) - glm::dvec3(d.centre), f);
        });
    }
    const double thrust_impulse = static_cast<double>(kH) * d.thrust;
    const double torque_impulse = static_cast<double>(kH) * d.torque;
    std::printf("[ dense ] D1's disc: thrust %.5f N, torque %.6f N m from its increments\n",
                -momentum.y / static_cast<double>(kH), -angular.y / static_cast<double>(kH));
    EXPECT_NEAR(momentum.y, -thrust_impulse, 1.0e-5 * thrust_impulse);
    EXPECT_NEAR(momentum.x, 0.0, 1.0e-6 * thrust_impulse);
    EXPECT_NEAR(momentum.z, 0.0, 1.0e-6 * thrust_impulse);
    EXPECT_NEAR(angular.y, -torque_impulse, 1.0e-4 * std::fabs(torque_impulse));
    EXPECT_NEAR(angular.x, 0.0, 1.0e-4 * std::fabs(torque_impulse));
    EXPECT_NEAR(angular.z, 0.0, 1.0e-4 * std::fabs(torque_impulse));
}

// The largest single-face increment for D1's disc at h 0.38 ms: 5.157 m/s a
// step measured for 10 rings of 64, 13.59 for 3 rings of 16 (triage 2). A
// bound, not the value: under 6 m/s.
TEST(AirflowDenseDisc, TheLargestFaceIncrementForD1sDiscIsUnderSixMetresASecond) {
    const Increments inc = spread(d1_disc(), kBox);
    float largest = 0.0f;
    for (const std::vector<float>* c : {&inc.du, &inc.dv, &inc.dw}) {
        for (const float x : *c) largest = std::max(largest, std::fabs(x));
    }
    std::printf("[ dense ] D1's disc: the largest single-face increment %.4f m/s per step (h %.2f ms)\n",
                static_cast<double>(largest), static_cast<double>(kH) * 1e3);
    EXPECT_LT(largest, 6.0f);
}

// ===========================================================================
// AirflowDiscPeak (opt-in)
// ===========================================================================

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)  // getenv
#endif
[[nodiscard]] bool disc_peak_enabled() {
    const char* e = std::getenv("SPADE_DISC_PEAK");
    return e != nullptr && *e != '\0' && std::strcmp(e, "0") != 0;
}
[[nodiscard]] uint32_t disc_peak_steps() {
    const char* e = std::getenv("SPADE_DISC_PEAK_STEPS");
    if (e == nullptr || *e == '\0') return 40;
    return static_cast<uint32_t>(std::strtoul(e, nullptr, 10));
}
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

// B7's physical bound in D1 (b7_harness.hpp, physical_bound(): 54.98 m/s).
constexpr double kPeakBound = 55.0;

// D1 with its solids, B7's box (b7_harness.hpp, d1_box()): the discs spread
// and fluid_step() itself, as LiveRun steps them, from rest. Each step's
// peak |u|, |v|, |w| and its faces over 55 m/s, logged; the peak under 55 at
// every step. Before the steps: each disc's points against the measured
// rings, and the forcing the four discs add in a step. About 20-26 s a step.
TEST(AirflowDiscPeak, D1WithSolidsStaysUnderFiftyFiveMetresASecond) {
    if (!disc_peak_enabled()) GTEST_SKIP() << "opt-in: SPADE_DISC_PEAK=1 (D1 with its solids, about 20-26 s a step)";
    const uint32_t steps = disc_peak_steps();
    spade::Result<b7::B7Box> box = b7::d1_box(b7::BoxSolids::with);
    ASSERT_TRUE(box.has_value()) << box.error().context;
    const GridShape& s = box->grid.shape;
    std::printf("disc-peak: %s; %zu discs, %u steps, h %.2f ms, the bound %.2f m/s (B7's %.2f)\n", box->config.c_str(),
                box->discs.size(), steps, static_cast<double>(box->cfg.h) * 1e3, kPeakBound, b7::physical_bound(*box));
    for (std::size_t n = 0; n < box->discs.size(); ++n) {
        const ActuatorDisc& d = box->discs[n];
        const DiscSpread sp = disc_spread(d, s, box->cfg.rho, box->cfg.h);
        std::printf("disc-peak: disc %zu at (%.4f, %.4f, %.4f) cells, axis (%.6f, %.6f, %.6f), T %.5f N, Q %.6f N m; "
                    "points off the measured rings: %zu of %u\n",
                    n, static_cast<double>(d.centre.x / s.dx), static_cast<double>(d.centre.y / s.dx),
                    static_cast<double>(d.centre.z / s.dx), static_cast<double>(d.axis.x),
                    static_cast<double>(d.axis.y), static_cast<double>(d.axis.z), static_cast<double>(d.thrust),
                    static_cast<double>(d.torque), off_the_measured_rings(d, sp), kDiscPoints);
    }
    {
        Increments f{std::vector<float>(box->grid.u.size(), 0.0f), std::vector<float>(box->grid.v.size(), 0.0f),
                     std::vector<float>(box->grid.w.size(), 0.0f)};
        for (const ActuatorDisc& d : box->discs) spread_actuator(d, s, box->cfg.rho, box->cfg.h, f.du, f.dv, f.dw);
        const auto largest = [](const std::vector<float>& c) {
            float m = 0.0f;
            for (const float x : c) m = std::max(m, std::fabs(x));
            return static_cast<double>(m);
        };
        double sum_v = 0.0;
        for (const float x : f.dv) sum_v += static_cast<double>(x);
        const double cell = static_cast<double>(s.dx) * s.dx * s.dx;
        std::printf("disc-peak: forcing: max|du| %.4f max|dv| %.4f max|dw| %.4f m/s per step; sum dv rho dx^3 / h = "
                    "%.5f N\n",
                    largest(f.du), largest(f.dv), largest(f.dw),
                    sum_v * static_cast<double>(box->cfg.rho) * cell / static_cast<double>(box->cfg.h));
    }
    MacGrid g = box->grid;
    spade::Result<FluidScratch> made = b7::box_scratch(*box);
    ASSERT_TRUE(made.has_value()) << made.error().context;
    FluidScratch& sc = *made;
    double top = 0.0;
    uint32_t top_at = 0;
    for (uint32_t n = 1; n <= steps; ++n) {
        const auto t0 = std::chrono::steady_clock::now();
        for (const ActuatorDisc& d : box->discs) spread_actuator(d, s, box->cfg.rho, box->cfg.h, sc.du, sc.dv, sc.dw);
        const auto t1 = std::chrono::steady_clock::now();
        const FluidHealth hl = fluid_step(g, box->bc, box->cfg, sc);
        const auto t2 = std::chrono::steady_clock::now();
        ASSERT_EQ(hl.fault, FluidFault::none) << "step " << n;
        double peak[3] = {0.0, 0.0, 0.0};
        std::size_t over = 0;
        const std::vector<float>* comp[3] = {&g.u, &g.v, &g.w};
        for (uint32_t c = 0; c < 3; ++c) {
            for (const float x : *comp[c]) {
                const double a = std::fabs(static_cast<double>(x));
                peak[c] = std::isfinite(a) ? std::max(peak[c], a) : a;
                over += !(a <= kPeakBound) ? 1u : 0u;
            }
        }
        double step_peak = 0.0;
        for (const double p : peak) {
            step_peak = std::isfinite(step_peak) && std::isfinite(p) ? std::max(step_peak, p)
                                                                     : std::numeric_limits<double>::infinity();
        }
        if (!(step_peak <= top)) {
            top = step_peak;
            top_at = n;
        }
        std::printf("disc-peak: step %u (t %.2f ms): |u| %.4f |v| %.4f |w| %.4f, peak %.4f m/s; over %.0f m/s: %zu "
                    "faces; CG %u it; spread %.0f ms, fluid_step %.0f ms\n",
                    n, static_cast<double>(box->cfg.h) * 1e3 * n, peak[0], peak[1], peak[2], step_peak, kPeakBound,
                    over, hl.pressure_iterations, std::chrono::duration<double, std::milli>(t1 - t0).count(),
                    std::chrono::duration<double, std::milli>(t2 - t1).count());
        std::fflush(stdout);
        EXPECT_LT(step_peak, kPeakBound) << "step " << n;
        if (!std::isfinite(step_peak)) break;
    }
    std::printf("disc-peak: summary: the peak %.4f m/s at step %u of %u\n", top, top_at, steps);
}

}  // namespace
