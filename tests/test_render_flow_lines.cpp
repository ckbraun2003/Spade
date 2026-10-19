// render/flow_lines.hpp: RK4 streamlines in arc length, against analytic
// fields. The volumes hold linear fields with dyadic coefficients, so f16
// stores them exactly and trilinear sampling reproduces them exactly: the
// only error left is the integrator's, and its order is measured.
// Test-only double; this file feeds no golden.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include <glm/glm.hpp>

#include "flow_synthetic.hpp"
#include "render/flow_lines.hpp"

namespace {

using spade::render::LineEnd;
using spade::render::Streamline;
using spade::render::StreamlineOptions;
using spade::render::trace_streamlines;

// A box [-4, 4]^3 of unit cells holding u = A x + b.
struct Linear {
    glm::dmat3 a;  // column-major: a[col][row]
    glm::dvec3 b;
    [[nodiscard]] glm::dvec3 at(glm::dvec3 x) const { return a * x + b; }
};

[[nodiscard]] spade::FlowVolume linear_volume(const Linear& field) {
    return spade::test::make_flow_volume(glm::vec3(-4.0f), 1.0f, 8, 8, 8, [field](glm::vec3 p) {
        const glm::dvec3 u = field.at(glm::dvec3(p));
        spade::test::FlowChannels c{};
        c[spade::kFlowU] = static_cast<float>(u.x);
        c[spade::kFlowV] = static_cast<float>(u.y);
        c[spade::kFlowW] = static_cast<float>(u.z);
        return c;
    });
}

// Rows (0.25, -1, 0.125), (0.875, -0.125, 0.25), (0, 0.25, -0.125); b dyadic.
[[nodiscard]] Linear generic_field() {
    return Linear{glm::transpose(glm::dmat3(0.25, -1.0, 0.125, 0.875, -0.125, 0.25, 0.0, 0.25, -0.125)),
                  glm::dvec3(0.5, 0.25, 0.125)};
}

[[nodiscard]] Streamline one_line(const spade::FlowVolume& volume, glm::vec3 seed, const StreamlineOptions& options) {
    const auto lines = trace_streamlines(volume, std::span<const glm::vec3>(&seed, 1), options);
    EXPECT_TRUE(lines.has_value()) << lines.error().context;
    EXPECT_EQ(lines->size(), 1u);
    return lines->empty() ? Streamline{} : lines->front();
}

TEST(RenderFlowLines, UniformFlowGivesAStraightEvenlySpacedLine) {
    const spade::FlowVolume volume = linear_volume(Linear{glm::dmat3(0.0), glm::dvec3(0.75, 0.5, 0.0)});
    const glm::vec3 seed(0.25f, -0.5f, 0.125f);
    const Streamline line = one_line(volume, seed, StreamlineOptions{});
    ASSERT_GT(line.points.size(), 20u);
    const glm::vec3 dir = glm::normalize(glm::vec3(0.75f, 0.5f, 0.0f));
    for (size_t i = 0; i < line.points.size(); ++i) {
        const float s = 0.25f * (static_cast<float>(i) - static_cast<float>(line.seed_point));
        EXPECT_LT(glm::length(line.points[i] - (seed + s * dir)), 1e-5f) << "point " << i;
        EXPECT_NEAR(line.speeds[i], std::sqrt(0.75f * 0.75f + 0.25f), 1e-6f);
    }
    EXPECT_EQ(line.points[line.seed_point], seed);
    EXPECT_EQ(line.backward_end, LineEnd::left_box);
    EXPECT_EQ(line.forward_end, LineEnd::left_box);
}

// The reference: the same ODE in double at a ten-thousandth of a cell.
[[nodiscard]] glm::dvec3 reference_end(const Linear& field, glm::dvec3 x, double length) {
    const auto f = [&](glm::dvec3 p) { return glm::normalize(field.at(p)); };
    const double h = 1e-4;
    for (int i = 0; i < static_cast<int>(std::lround(length / h)); ++i) {
        const glm::dvec3 k1 = f(x), k2 = f(x + 0.5 * h * k1), k3 = f(x + 0.5 * h * k2), k4 = f(x + h * k3);
        x += h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
    return x;
}

// Fourth order: halving the step cuts the end-point error about sixteen
// times. Observed order >= 3.5 on both pairs (measured 4.48 and 4.06 in a
// numpy port, 2026-10-06); a second-order slip gives about 2.
TEST(RenderFlowLines, Rk4IsFourthOrder) {
    const Linear field = generic_field();
    const spade::FlowVolume volume = linear_volume(field);
    const glm::vec3 seed(0.3f, -0.4f, 0.2f);
    const double length = 3.0;
    const glm::dvec3 want = reference_end(field, glm::dvec3(seed), length);
    double err[3];
    const float steps[3] = {0.5f, 0.25f, 0.125f};
    for (int k = 0; k < 3; ++k) {
        StreamlineOptions options;
        options.step_cells = steps[k];
        options.backward = false;
        options.max_steps = static_cast<uint32_t>(std::lround(length / steps[k]));
        const Streamline line = one_line(volume, seed, options);
        ASSERT_EQ(line.forward_end, LineEnd::step_budget);
        err[k] = glm::length(glm::dvec3(line.points.back()) - want);
    }
    const double p1 = std::log2(err[0] / err[1]), p2 = std::log2(err[1] / err[2]);
    std::printf("[ RK4 order ] errors %.3g %.3g %.3g, observed order %.2f %.2f\n", err[0], err[1], err[2], p1, p2);
    EXPECT_GE(p1, 3.5);
    EXPECT_GE(p2, 3.5);
}

// Solid-body rotation about z: streamlines are circles. The radius holds
// along a whole revolution (RK4's drift there is h^5).
TEST(RenderFlowLines, SolidBodyRotationGivesCircles) {
    const spade::FlowVolume volume =
        linear_volume(Linear{glm::dmat3(glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(-1.0, 0.0, 0.0), glm::dvec3(0.0)),
                             glm::dvec3(0.0)});
    const float r = 2.0f;
    StreamlineOptions options;
    options.backward = false;
    options.max_steps = static_cast<uint32_t>(std::lround(2.0 * 3.14159265358979 * r / 0.25));
    const Streamline line = one_line(volume, glm::vec3(r, 0.0f, 0.5f), options);
    ASSERT_EQ(line.points.size(), options.max_steps + 1u);
    double worst = 0.0;
    for (const glm::vec3& p : line.points) {
        worst = std::max(worst, std::fabs(std::hypot(double{p.x}, double{p.y}) - r));
        EXPECT_EQ(p.z, 0.5f);
    }
    EXPECT_LT(worst, 1e-5) << "metres off the circle";
    EXPECT_LT(glm::length(line.points.back() - line.points.front()), 0.25f) << "a revolution comes back round";
}

// A live view's cap (the lead, 2026-10-19): each direction stops after its
// length in box diagonals, so a closed orbit costs one crossing of the box,
// not its whole step budget. The box's diagonal is 8 sqrt(3) m, 55 steps of
// 0.25 m. The default has no cap, so the films and goldens are unchanged.
TEST(RenderFlowLines, ALengthCapStopsEachDirectionAfterItsDiagonals) {
    const spade::FlowVolume volume =
        linear_volume(Linear{glm::dmat3(glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(-1.0, 0.0, 0.0), glm::dvec3(0.0)),
                             glm::dvec3(0.0)});
    const glm::vec3 seed(2.0f, 0.0f, 0.5f);
    const Streamline uncapped = one_line(volume, seed, StreamlineOptions{});
    ASSERT_EQ(uncapped.points.size(), 2u * 4096u + 1u) << "a circle runs its whole budget";
    StreamlineOptions capped;
    capped.max_length_diagonals = 1.0f;
    const Streamline line = one_line(volume, seed, capped);
    ASSERT_EQ(line.points.size(), 2u * 55u + 1u);
    EXPECT_EQ(line.seed_point, 55u);
    EXPECT_EQ(line.backward_end, LineEnd::step_budget);
    EXPECT_EQ(line.forward_end, LineEnd::step_budget);
    for (std::size_t i = 0; i < line.points.size(); ++i) {
        EXPECT_EQ(line.points[i], uncapped.points[4096u - 55u + i]) << "the same line, cut short, at " << i;
    }
    capped.max_length_diagonals = 0.5f;
    EXPECT_EQ(one_line(volume, seed, capped).points.size(), 2u * 27u + 1u);
    capped.max_length_diagonals = 1.0f;
    capped.max_steps = 10;
    EXPECT_EQ(one_line(volume, seed, capped).points.size(), 21u) << "a budget under the cap still ends it";
}

TEST(RenderFlowLines, TheLivePresetCapsAtOneDiagonalAndKeepsTheRest) {
    const StreamlineOptions live = StreamlineOptions::live();
    EXPECT_EQ(live.max_length_diagonals, 1.0f);
    StreamlineOptions rest = live;
    rest.max_length_diagonals = 0.0f;
    EXPECT_EQ(rest, StreamlineOptions{});
    EXPECT_EQ(StreamlineOptions{}.max_length_diagonals, 0.0f) << "the default has no cap";
}

// L6: each way a line ends is recorded.
TEST(RenderFlowLines, EveryEndIsRecorded) {
    // Flow toward +x on the left half, still on the right: the line stalls.
    const spade::FlowVolume stall = spade::test::make_flow_volume(glm::vec3(-4.0f), 1.0f, 8, 8, 8, [](glm::vec3 p) {
        spade::test::FlowChannels c{};
        c[spade::kFlowU] = p.x < 0.0f ? 1.0f : 0.0f;
        return c;
    });
    const Streamline stalled = one_line(stall, glm::vec3(-2.0f, 0.0f, 0.0f), StreamlineOptions{});
    EXPECT_EQ(stalled.forward_end, LineEnd::stagnation);
    EXPECT_EQ(stalled.backward_end, LineEnd::left_box);

    const spade::FlowVolume holed = spade::test::make_flow_volume(glm::vec3(-4.0f), 1.0f, 8, 8, 8, [](glm::vec3 p) {
        spade::test::FlowChannels c{};
        c[spade::kFlowU] = p.x > 1.0f ? std::numeric_limits<float>::quiet_NaN() : 1.0f;
        return c;
    });
    EXPECT_EQ(one_line(holed, glm::vec3(-2.0f, 0.0f, 0.0f), StreamlineOptions{}).forward_end, LineEnd::no_data);

    const spade::FlowVolume uniform = linear_volume(Linear{glm::dmat3(0.0), glm::dvec3(1.0, 0.0, 0.0)});
    StreamlineOptions short_budget;
    short_budget.max_steps = 3;
    const Streamline budget = one_line(uniform, glm::vec3(0.0f), short_budget);
    EXPECT_EQ(budget.forward_end, LineEnd::step_budget);
    EXPECT_EQ(budget.backward_end, LineEnd::step_budget);
    EXPECT_EQ(budget.points.size(), 7u) << "three steps each way and the seed";

    const Streamline outside = one_line(uniform, glm::vec3(9.0f, 0.0f, 0.0f), StreamlineOptions{});
    EXPECT_EQ(outside.points.size(), 1u);
    EXPECT_EQ(outside.forward_end, LineEnd::left_box);
    EXPECT_EQ(outside.backward_end, LineEnd::left_box);
}

TEST(RenderFlowLines, BadInputIsRefused) {
    const spade::FlowVolume volume = linear_volume(Linear{glm::dmat3(0.0), glm::dvec3(1.0, 0.0, 0.0)});
    const glm::vec3 seed(0.0f);
    const auto refused = [&](std::span<const glm::vec3> seeds, const StreamlineOptions& options, const char* what) {
        const auto r = trace_streamlines(volume, seeds, options);
        ASSERT_FALSE(r.has_value()) << what << " must be refused";
        EXPECT_EQ(r.error().code, spade::Code::invalid_argument) << what;
    };
    const glm::vec3 nan_seed(std::numeric_limits<float>::quiet_NaN());
    refused(std::span<const glm::vec3>(&nan_seed, 1), {}, "a NaN seed");
    StreamlineOptions o;
    o.step_cells = 0.0f;
    refused(std::span<const glm::vec3>(&seed, 1), o, "a zero step");
    o = {};
    o.step_cells = 5.0f;
    refused(std::span<const glm::vec3>(&seed, 1), o, "a step past four cells");
    o = {};
    o.max_steps = 0;
    refused(std::span<const glm::vec3>(&seed, 1), o, "no step budget");
    o = {};
    o.stagnation_speed = -1.0f;
    refused(std::span<const glm::vec3>(&seed, 1), o, "a negative stagnation floor");
    o = {};
    o.forward = o.backward = false;
    refused(std::span<const glm::vec3>(&seed, 1), o, "neither direction");
    o = {};
    o.max_length_diagonals = -1.0f;
    refused(std::span<const glm::vec3>(&seed, 1), o, "a negative length cap");
    o.max_length_diagonals = std::numeric_limits<float>::quiet_NaN();
    refused(std::span<const glm::vec3>(&seed, 1), o, "a NaN length cap");
    spade::FlowVolume bad = volume;
    bad.header.cell = 0.0f;
    const auto r = trace_streamlines(bad, std::span<const glm::vec3>(&seed, 1), {});
    ASSERT_FALSE(r.has_value()) << "a malformed volume must be refused";
}

TEST(RenderFlowLines, SeedHelpersPlaceTheirPoints) {
    const auto rake = spade::render::seed_rake(glm::vec3(0.0f), glm::vec3(4.0f, 2.0f, 0.0f), 5);
    ASSERT_EQ(rake.size(), 5u);
    EXPECT_EQ(rake.front(), glm::vec3(0.0f));
    EXPECT_EQ(rake.back(), glm::vec3(4.0f, 2.0f, 0.0f));
    EXPECT_EQ(rake[2], glm::vec3(2.0f, 1.0f, 0.0f));
    EXPECT_EQ(spade::render::seed_rake(glm::vec3(0.0f), glm::vec3(2.0f), 1).front(), glm::vec3(1.0f));

    const glm::vec3 centre(1.0f, 2.0f, 3.0f), axis(0.0f, 1.0f, 0.0f);
    const auto ring = spade::render::seed_ring(centre, axis, 0.5f, 8);
    ASSERT_EQ(ring.size(), 8u);
    for (const glm::vec3& p : ring) {
        EXPECT_NEAR(glm::length(p - centre), 0.5f, 1e-6f);
        EXPECT_NEAR(glm::dot(p - centre, axis), 0.0f, 1e-6f);
    }
    EXPECT_GT(glm::length(ring[0] - ring[4]), 0.99f) << "opposite points are a diameter apart";
}

}  // namespace
