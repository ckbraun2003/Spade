// The airflow solver core's fluid step and its golden (airflow core plan,
// Task 8).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"

namespace {

using namespace spade::physics::airflow;

struct Lcg {
    uint32_t state = 12345u;
    float next() noexcept {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }
};

[[nodiscard]] uint64_t digest(const MacGrid& g) {
    uint64_t h = 1469598103934665603ull;
    const auto fold = [&](const std::vector<float>& v) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(v.data());
        for (std::size_t i = 0; i < v.size() * sizeof(float); ++i) {
            h ^= bytes[i];
            h *= 1099511628211ull;
        }
    };
    fold(g.u);
    fold(g.v);
    fold(g.w);
    fold(g.p);
    return h;
}

[[nodiscard]] DomainBc cavity_bc() {
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    bc.wall_velocity[kYPlus] = glm::vec3(1.0f, 0.0f, 0.0f);
    return bc;
}

TEST(AirflowStep, TheConfigurationIsChecked) {
    const MacGrid g = *make_grid({4, 4, 4, 0.25f}, DomainBc{});
    const auto refused = [&](FluidConfig cfg, const char* what) {
        const auto s = make_fluid_scratch(g, DomainBc{}, cfg);
        if (s.has_value()) return testing::AssertionFailure() << "accepted";
        if (s.error().context.find(what) == std::string::npos) return testing::AssertionFailure() << s.error().context;
        return testing::AssertionSuccess();
    };
    EXPECT_TRUE(refused({.rho = 0.0f, .h = 0.01f}, "rho"));
    EXPECT_TRUE(refused({.h = 0.0f}, "h"));
    EXPECT_TRUE(refused({.h = std::numeric_limits<float>::infinity()}, "h"));
    EXPECT_TRUE(refused({.nu = -1.0f, .h = 0.01f}, "nu"));
    EXPECT_TRUE(refused({.h = 0.01f, .n_v = 0}, "n_v"));
    EXPECT_TRUE(refused({.nu = 0.01f, .h = 0.01f, .visc_sweeps = 0}, "visc_sweeps"));
    EXPECT_TRUE(make_fluid_scratch(g, DomainBc{}, {.h = 0.01f}).has_value());
}

TEST(AirflowStep, AUniformFlowThroughAPeriodicBoxIsExact) {
    DomainBc bc;
    bc.face.fill(FaceBc::periodic);
    MacGrid g = *make_grid({8, 8, 8, 0.125f}, bc);
    std::fill(g.u.begin(), g.u.end(), 1.0f);
    std::fill(g.v.begin(), g.v.end(), 0.5f);
    const MacGrid start = g;
    // The lean (inviscid) mode: the implicit viscous solve's fp32 rounding may
    // move a uniform field by an ulp, so viscosity is checked in AirflowViscous.
    const FluidConfig cfg{.h = 0.01f};
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    for (int n = 0; n < 10; ++n) fluid_step(g, bc, cfg, s);
    EXPECT_EQ(g.u, start.u);
    EXPECT_EQ(g.v, start.v);
    EXPECT_EQ(g.w, start.w);
    for (float p : g.p) EXPECT_EQ(p, 0.0f);
}

TEST(AirflowStep, TwoRunsAreBitIdentical) {
    const auto run = [] {
        const DomainBc bc = cavity_bc();
        MacGrid g = *make_grid({12, 12, 12, 1.0f / 12.0f}, bc);
        Lcg rng;
        for (float& x : g.u) x = 0.1f * rng.next();
        for (float& x : g.v) x = 0.1f * rng.next();
        for (float& x : g.w) x = 0.1f * rng.next();
        const FluidConfig cfg{.rho = 1.0f, .nu = 0.01f, .h = 0.5f / 12.0f};
        FluidScratch s = *make_fluid_scratch(g, bc, cfg);
        for (int n = 0; n < 32; ++n) fluid_step(g, bc, cfg, s);
        return digest(g);
    };
    EXPECT_EQ(run(), run());
}

// THE SOLVER CORE'S GOLDEN (TD-1, TD-12). A 16^3 lid-driven cavity at Re 100,
// 64 steps at CFL 0.5, folded by FNV-1a over u, v, w and p.
//   generated  2026-10-12 by this test, MSVC release (VS 2022), on physics/airflow-core at
//              3c4cb17 plus Task 8 (the commit that adds this test)
//   cross-check MSVC debug agrees; the gcc-13 Docker leg (TD-12) reproduces it on a fresh
//              -NoSeed volume at 742f56e (build-docker/742f56ea2a52, 2026-10-12): FINAL
// A change to any value here is a golden regeneration under TD-1, with its
// reason in the commit.
constexpr uint64_t kSolverCoreDigest = 0x7ea42593859fbcebull;

TEST(AirflowStep, TheSolverCoresDigestIsPinned) {
    const DomainBc bc = cavity_bc();
    MacGrid g = *make_grid({16, 16, 16, 1.0f / 16.0f}, bc);
    const FluidConfig cfg{.rho = 1.0f, .nu = 0.01f, .h = 0.5f / 16.0f, .n_v = 2, .visc_sweeps = 8};
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    for (int n = 0; n < 64; ++n) fluid_step(g, bc, cfg, s);
    for (float x : g.u) ASSERT_TRUE(std::isfinite(x));
    const uint64_t d = digest(g);
    std::printf("airflow solver core digest: 0x%016llx\n", static_cast<unsigned long long>(d));
    EXPECT_EQ(d, kSolverCoreDigest);
}


// The lead's folds into Task 8 (2026-10-05, 2026-10-06, 2026-10-12).

// (1) The byte cap counts the scratch too (make_grid's cap counts only the
// grid). A grid that fits under the cap but whose step does not is refused
// before anything is allocated: the grid here is a shape with no storage.
TEST(AirflowStep, AFluidScratchPastTheByteCapIsRefusedBeforeAllocating) {
    MacGrid g;
    g.shape = {512, 512, 256, 0.01f};  // no storage: make_fluid_scratch must refuse from the shape alone
    ASSERT_LE(grid_bytes(g.shape), kGridByteCap) << "the grid alone fits";
    ASSERT_GT(fluid_bytes(g.shape, DomainBc{}), kGridByteCap) << "the grid and its scratch do not";
    const auto s = make_fluid_scratch(g, DomainBc{}, {.h = 0.001f});
    ASSERT_FALSE(s.has_value());
    EXPECT_EQ(s.error().code, spade::Code::capacity_exceeded);
    EXPECT_NE(s.error().context.find(std::to_string(fluid_bytes(g.shape, DomainBc{}))), std::string::npos)
        << s.error().context;
    EXPECT_NE(s.error().context.find(std::to_string(kGridByteCap)), std::string::npos) << s.error().context;
}

// (2) A closed domain (no outlet) cannot take a net prescribed flux through
// its walls: remove_mean would quietly cancel the imbalance. Refused at setup
// naming the faces, and faulted in a step whose configuration turns so.
TEST(AirflowStep, AClosedDomainWhoseWallsPushANetFluxIsRefusedNamingTheFaces) {
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    bc.wall_velocity[kXMinus] = glm::vec3(0.5f, 0.0f, 0.0f);  // air pushed in through x-, nowhere to go
    MacGrid g = *make_grid({8, 8, 8, 0.125f}, bc);
    const auto s = make_fluid_scratch(g, bc, {.h = 0.01f});
    ASSERT_FALSE(s.has_value());
    EXPECT_EQ(s.error().code, spade::Code::invalid_argument);
    EXPECT_NE(s.error().context.find("x-"), std::string::npos) << s.error().context;

    DomainBc piston = bc;
    piston.wall_velocity[kXPlus] = glm::vec3(0.5f, 0.0f, 0.0f);  // in at x-, out at x+: balanced
    ASSERT_TRUE(make_fluid_scratch(g, piston, {.h = 0.01f}).has_value());

    FluidScratch ok = *make_fluid_scratch(g, piston, {.h = 0.01f});
    const MacGrid before = g;
    const FluidHealth h = fluid_step(g, bc, {.h = 0.01f}, ok);
    EXPECT_EQ(h.fault, FluidFault::ill_posed_domain);
    EXPECT_NE(describe(h, g.shape).find("x-"), std::string::npos) << describe(h, g.shape);
    EXPECT_EQ(g.u, before.u) << "a refused step changes nothing";
}

// The final review's I1 (2026-10-12): an open face the ambient enters is a
// prescribed flux too. A box walled on five sides with the wind blowing in
// through the sixth has no outlet, so its inflow has nowhere to go: refused
// at setup, and faulted mid-run when a reversing gust makes that face an
// inflow (before, the mean removal hid it as a uniform sink).
TEST(AirflowStep, AClosedDomainWithAnInflowFaceAndNoOutletIsRefusedNamingTheFace) {
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    bc.face[kXMinus] = FaceBc::open;
    bc.ambient = glm::vec3(1.0f, 0.0f, 0.0f);  // entering through x-: an inflow, and no outlet anywhere
    MacGrid g = *make_grid({8, 8, 8, 0.125f}, bc);
    const auto s = make_fluid_scratch(g, bc, {.h = 0.01f});
    ASSERT_FALSE(s.has_value());
    EXPECT_EQ(s.error().code, spade::Code::invalid_argument);
    EXPECT_NE(s.error().context.find("x-"), std::string::npos) << s.error().context;

    DomainBc leaving = bc;
    leaving.ambient = glm::vec3(-1.0f, 0.0f, 0.0f);  // leaving through x-: an outlet, well posed
    FluidScratch ok = *make_fluid_scratch(g, leaving, {.h = 0.01f});
    ASSERT_EQ(fluid_step(g, leaving, {.h = 0.01f}, ok).fault, FluidFault::none);
    const MacGrid before = g;
    const FluidHealth h = fluid_step(g, bc, {.h = 0.01f}, ok);  // the gust reverses
    EXPECT_EQ(h.fault, FluidFault::ill_posed_domain);
    EXPECT_NE(describe(h, g.shape).find("x-"), std::string::npos) << describe(h, g.shape);
    EXPECT_EQ(g.u, before.u) << "a refused step changes nothing";
}

// (3) The health check: a non-finite value anywhere stops the step and names
// the first bad sample in a fixed order (u, v, w, p; array order), and the
// scratch stays poisoned: the next step refuses and changes nothing.
TEST(AirflowStep, ANonFiniteValueStopsTheStepAndNamesItsSample) {
    const DomainBc bc;
    MacGrid g = *make_grid({8, 8, 8, 0.125f}, bc);
    g.u[u_index(g.shape, 3, 4, 5)] = std::numeric_limits<float>::quiet_NaN();
    const FluidConfig cfg{.h = 0.01f};
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    const FluidHealth first = fluid_step(g, bc, cfg, s);
    ASSERT_EQ(first.fault, FluidFault::non_finite);
    const std::string why = describe(first, g.shape);
    EXPECT_NE(why.find("non-finite"), std::string::npos) << why;
    const MacGrid after = g;
    const FluidHealth second = fluid_step(g, bc, cfg, s);
    EXPECT_EQ(second.fault, FluidFault::non_finite) << "poisoned";
    EXPECT_EQ(second.index, first.index);
    EXPECT_EQ(std::memcmp(g.u.data(), after.u.data(), g.u.size() * sizeof(float)), 0) << "a poisoned step changes nothing";
}

TEST(AirflowStep, AHealthyStepReportsNoFault) {
    const DomainBc bc;
    MacGrid g = *make_grid({8, 8, 8, 0.125f}, bc);
    const FluidConfig cfg{.h = 0.01f};
    FluidScratch s = *make_fluid_scratch(g, bc, cfg);
    EXPECT_EQ(fluid_step(g, bc, cfg, s).fault, FluidFault::none);
}

TEST(AirflowActuator, TheKernelSumsToOneAlongAnAxis) {
    Lcg rng;
    for (int n = 0; n < 100; ++n) {
        const float x = 0.5f * (rng.next() + 1.0f);  // in [0, 1)
        float sum = 0.0f;
        for (int k = -2; k <= 3; ++k) sum += peskin4(x - static_cast<float>(k));
        EXPECT_NEAR(sum, 1.0f, 1.0e-6f) << x;
    }
}

TEST(AirflowActuator, TheDiscAddsExactlyItsImpulse) {
    const GridShape shape{32, 32, 32, 1.0f / 32.0f};
    const MacGrid g = *make_grid(shape, DomainBc{});
    const ActuatorDisc disc{.centre = {0.5f, 0.5f, 0.5f}, .axis = {0.0f, -1.0f, 0.0f}, .radius = 0.2f, .thrust = 2.0f};
    ASSERT_TRUE(check_disc(disc, shape).has_value());
    std::vector<float> du(g.u.size(), 0.0f), dv(g.v.size(), 0.0f), dw(g.w.size(), 0.0f);
    const float rho = 1.225f;
    const float h = 0.004f;
    spread_actuator(disc, shape, rho, h, du, dv, dw);
    const double cell = static_cast<double>(shape.dx) * shape.dx * shape.dx;
    const auto momentum = [&](const std::vector<float>& d) {
        double m = 0.0;
        for (float x : d) m += static_cast<double>(x) * rho * cell;
        return m;
    };
    const double expected = -static_cast<double>(h) * 2.0;  // along -y
    EXPECT_NEAR(momentum(dv), expected, 1.0e-5 * std::fabs(expected));
    EXPECT_EQ(momentum(du), 0.0) << "the axis has no x component";
    EXPECT_EQ(momentum(dw), 0.0);
}

// Review Focus 2.
TEST(AirflowActuator, ADiscWhoseSupportLeavesTheGridIsRefused) {
    const GridShape shape{32, 32, 32, 1.0f / 32.0f};
    const auto refused = [&](const ActuatorDisc& d, const char* what) {
        const auto r = check_disc(d, shape);
        if (r.has_value()) return testing::AssertionFailure() << "accepted";
        if (r.error().context.find(what) == std::string::npos) return testing::AssertionFailure() << r.error().context;
        return testing::AssertionSuccess();
    };
    const ActuatorDisc ok{.centre = {0.5f, 0.5f, 0.5f}, .axis = {0.0f, -1.0f, 0.0f}, .radius = 0.2f, .thrust = 1.0f};
    ActuatorDisc edge = ok;
    edge.centre.x = 0.1f;
    EXPECT_TRUE(refused(edge, "leaves the grid"));
    ActuatorDisc long_axis = ok;
    long_axis.axis = {0.0f, -2.0f, 0.0f};
    EXPECT_TRUE(refused(long_axis, "unit"));
    ActuatorDisc flat = ok;
    flat.radius = 0.0f;
    EXPECT_TRUE(refused(flat, "radius"));
    ActuatorDisc wild = ok;
    wild.torque = std::numeric_limits<float>::infinity();
    EXPECT_TRUE(refused(wild, "torque"));
}

// P1 (the demonstration plan; spec §3.1): the swirl. The disc's torque goes to
// the air as tangential forces about its axis, torque / (n r) at each of its n
// points on the ring of radius r (constant circulation: the ideal rotor under the
// disc's uniform thrust loading), so the points' sum of r |f| is the torque.
// Peskin's kernel keeps first moments, so the air's angular momentum about
// the centre gains h * torque along the axis, by the right-hand rule, and its
// linear momentum only the thrust's impulse.
TEST(AirflowActuator, ASwirlingDiscAddsItsTorqueAboutItsAxisAndNoSideForce) {
    const GridShape shape{32, 32, 32, 1.0f / 32.0f};
    const MacGrid g = *make_grid(shape, DomainBc{});
    const float rho = 1.225f;
    const float h = 0.004f;
    for (const float torque : {0.05f, -0.05f}) {
        const ActuatorDisc disc{.centre = {0.5f, 0.5f, 0.5f}, .axis = {0.0f, -1.0f, 0.0f}, .radius = 0.2f,
                                .thrust = 2.0f, .torque = torque};
        ASSERT_TRUE(check_disc(disc, shape).has_value());
        std::vector<float> du(g.u.size(), 0.0f), dv(g.v.size(), 0.0f), dw(g.w.size(), 0.0f);
        spread_actuator(disc, shape, rho, h, du, dv, dw);
        const double cell = static_cast<double>(shape.dx) * shape.dx * shape.dx;
        const std::vector<float>* d[3] = {&du, &dv, &dw};
        glm::dvec3 momentum(0.0);
        glm::dvec3 angular(0.0);
        for (uint32_t c = 0; c < 3; ++c) {
            for_each_sample(shape, c, [&](std::size_t idx, glm::vec3 x) {
                const double m = static_cast<double>((*d[c])[idx]) * rho * cell;
                glm::dvec3 f(0.0);
                f[static_cast<glm::length_t>(c)] = m;
                momentum += f;
                angular += glm::cross(glm::dvec3(x) - glm::dvec3(disc.centre), f);
            });
        }
        const double thrust_impulse = static_cast<double>(h) * 2.0;
        const double torque_impulse = static_cast<double>(h) * torque;
        EXPECT_NEAR(momentum.y, -thrust_impulse, 1.0e-5 * thrust_impulse) << torque;
        EXPECT_NEAR(momentum.x, 0.0, 1.0e-6 * thrust_impulse) << "no side force";
        EXPECT_NEAR(momentum.z, 0.0, 1.0e-6 * thrust_impulse) << "no side force";
        EXPECT_NEAR(angular.y, -torque_impulse, 1.0e-4 * std::fabs(torque_impulse)) << "about the axis (0, -1, 0)";
        EXPECT_NEAR(angular.x, 0.0, 1.0e-4 * std::fabs(torque_impulse));
        EXPECT_NEAR(angular.z, 0.0, 1.0e-4 * std::fabs(torque_impulse));
    }
}

}  // namespace
