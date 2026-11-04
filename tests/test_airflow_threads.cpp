// ---------------------------------------------------------------------------
// test_airflow_threads.cpp -- the CPU fluid step on threads (L18; Rendering
// builds it in Physics' airflow files, Physics reviews). FluidConfig::threads
// splits advect_velocity() and diffuse() across a WorkerPool by k plane, and
// spread_actuators() keeps each disc's weight sums: the bits must not move.
//
//   AirflowThreads  fluid_step() at 1, 2, 3 and 8 threads, and on a pool that
//                   runs every split's items last to first, u, v, w and p
//                   compared by memcmp and the health field by field, on the
//                   odd open box (25x21x19), channels periodic along x and
//                   along z (make_grid() takes only even periodic axes) and
//                   a box with a sphere; spread_actuators() against
//                   spread_actuator() disc by disc while discs move, swirl and
//                   stop swirling, from increments holding -0.
//
// The reversed pool makes an order-dependent split fail every run, where on
// threads it would only race (the caller takes item 0 first, so a race
// mostly lands on the serial answer).
//
// Mutations: one rhs shared by the threads (the open box goes red); both
// colours of a sweep in one pass, no barrier between them (the open box goes
// red); a kept +0 sum dropped (the spread test goes red). Physics' third, a
// same-colour wrap split across threads, cannot be built: make_grid()
// refuses an odd periodic axis.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>
#include <span>
#include <vector>

#include <glm/glm.hpp>

#include "airflow_solid_shapes.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"

namespace {

using namespace spade::physics::airflow;

struct Lcg {
    uint32_t state = 2463534242u;
    float next() noexcept {
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }
};

struct Case {
    GridShape shape;
    DomainBc bc;
    bool sphere = false;
};

struct Run {
    MacGrid g;
    std::vector<FluidHealth> health;
};

[[nodiscard]] std::vector<ActuatorDisc> discs_at(const GridShape& s, uint32_t step) {
    const glm::vec3 mid(0.5f * static_cast<float>(s.nx) * s.dx, 0.5f * static_cast<float>(s.ny) * s.dx,
                        0.5f * static_cast<float>(s.nz) * s.dx);
    std::vector<ActuatorDisc> d(2);
    d[0].centre = mid + glm::vec3(-2.5f * s.dx, 2.0f * s.dx, 0.25f * s.dx);
    d[0].radius = 2.2f * s.dx;
    d[0].thrust = 0.4f + 0.05f * static_cast<float>(step);
    d[0].torque = 0.01f;
    d[1].centre = mid + glm::vec3(2.5f * s.dx + 0.1f * s.dx * static_cast<float>(step), 2.0f * s.dx, -0.5f * s.dx);
    d[1].axis = glm::normalize(glm::vec3(0.2f, -1.0f, 0.1f));
    d[1].radius = 2.0f * s.dx;
    d[1].thrust = 0.3f;
    d[1].torque = step % 3u == 2u ? 0.0f : -0.012f;
    return d;
}

[[nodiscard]] Run run(const Case& c, uint32_t threads, uint32_t steps, bool reversed = false) {
    Run out;
    spade::Result<MacGrid> grid = make_grid(c.shape, c.bc);
    if (!grid) {
        ADD_FAILURE() << grid.error().context;
        return out;
    }
    out.g = std::move(*grid);
    Lcg rng;
    for (std::vector<float>* a : {&out.g.u, &out.g.v, &out.g.w}) {
        for (float& x : *a) x = 0.5f * rng.next();
    }
    FluidConfig cfg{.rho = 1.225f, .nu = 1.5e-5f * 2000.0f, .h = 0.004f};
    cfg.threads = threads;
    spade::Result<FluidScratch> scratch = make_fluid_scratch(out.g, c.bc, cfg);
    if (!scratch) {
        ADD_FAILURE() << scratch.error().context;
        return out;
    }
    FluidScratch& s = *scratch;
    EXPECT_EQ(s.pool != nullptr, threads > 1u);
    if (reversed) s.pool = std::make_shared<WorkerPool>(threads, ItemOrder::reversed);
    if (c.sphere) {
        airflow_test::Scene scene;
        const float dx = c.shape.dx;
        scene.shapes.push_back(airflow_test::Shape{.name = "sphere",
                                                   .kind = airflow_test::Kind::sphere,
                                                   .centre = glm::vec3(11.0f * dx, 7.0f * dx, 9.0f * dx),
                                                   .radius = 3.5f * dx,
                                                   .min_feature = 7.0f * dx});
        EXPECT_TRUE(set_solids(s, out.g, c.bc, scene.parts(), scene.sampler()).has_value()) << s.solids.run_line;
    }
    for (uint32_t step = 0; step < steps; ++step) {
        const std::vector<ActuatorDisc> d = discs_at(c.shape, step);
        spread_actuators(d, c.shape, cfg.rho, cfg.h, s);
        out.health.push_back(fluid_step(out.g, c.bc, cfg, s));
    }
    return out;
}

[[nodiscard]] bool same(const std::vector<float>& a, const std::vector<float>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
}

void expect_same(const Run& one, const Run& many, const char* what) {
    SCOPED_TRACE(what);
    EXPECT_TRUE(same(one.g.u, many.g.u)) << "u";
    EXPECT_TRUE(same(one.g.v, many.g.v)) << "v";
    EXPECT_TRUE(same(one.g.w, many.g.w)) << "w";
    EXPECT_TRUE(same(one.g.p, many.g.p)) << "p";
    ASSERT_EQ(one.health.size(), many.health.size());
    for (std::size_t i = 0; i < one.health.size(); ++i) {
        const FluidHealth& a = one.health[i];
        const FluidHealth& b = many.health[i];
        EXPECT_EQ(a.fault, b.fault) << "step " << i;
        EXPECT_EQ(a.quantity, b.quantity) << "step " << i;
        EXPECT_EQ(a.faces, b.faces) << "step " << i;
        EXPECT_EQ(a.index, b.index) << "step " << i;
        EXPECT_EQ(std::bit_cast<uint32_t>(a.pressure_residual), std::bit_cast<uint32_t>(b.pressure_residual))
            << "step " << i;
        EXPECT_EQ(a.pressure_iterations, b.pressure_iterations) << "step " << i;
        EXPECT_EQ(a.capped, b.capped) << "step " << i;
        EXPECT_EQ(a.unconverged, b.unconverged) << "step " << i;
        EXPECT_EQ(a.stalled, b.stalled) << "step " << i;
    }
}

void expect_same_at_every_count(const Case& c) {
    constexpr uint32_t kSteps = 6;
    const Run one = run(c, 1u, kSteps);
    ASSERT_EQ(one.health.size(), kSteps);
    expect_same(one, run(c, 2u, kSteps), "2 threads");
    expect_same(one, run(c, 3u, kSteps), "3 threads");
    expect_same(one, run(c, 8u, kSteps), "8 threads");
    expect_same(one, run(c, 3u, kSteps, true), "items reversed");
}

}  // namespace

// The odd open box: every face open, no axis even, the discs swirling.
TEST(AirflowThreads, TheOddOpenBoxStepsToTheSameBitsAtEveryThreadCount) {
    Case c;
    c.shape = GridShape{25, 21, 19, 0.02f};
    expect_same_at_every_count(c);
}

// A channel periodic along x: the wrap joins samples of one plane.
TEST(AirflowThreads, AnXChannelStepsToTheSameBitsAtEveryThreadCount) {
    Case c;
    c.shape = GridShape{16, 11, 9, 0.02f};
    c.bc.face[kXMinus] = FaceBc::periodic;
    c.bc.face[kXPlus] = FaceBc::periodic;
    c.bc.face[kYMinus] = FaceBc::wall;
    c.bc.face[kYPlus] = FaceBc::wall;
    expect_same_at_every_count(c);
}

// A channel periodic along z: the wrap joins planes 0 and n - 1, which
// different threads relax.
TEST(AirflowThreads, AZChannelStepsToTheSameBitsAtEveryThreadCount) {
    Case c;
    c.shape = GridShape{13, 11, 16, 0.02f};
    c.bc.face[kZMinus] = FaceBc::periodic;
    c.bc.face[kZPlus] = FaceBc::periodic;
    c.bc.face[kYMinus] = FaceBc::wall;
    c.bc.face[kYPlus] = FaceBc::wall;
    expect_same_at_every_count(c);
}

// A walled box with a sphere: the viscous solve's wall links and fixed
// samples, and the CG's projection.
TEST(AirflowThreads, ABoxWithASphereStepsToTheSameBitsAtEveryThreadCount) {
    Case c;
    c.shape = GridShape{22, 16, 18, 0.02f};
    c.bc.face.fill(FaceBc::wall);
    c.bc.face[kYPlus] = FaceBc::open;
    c.sphere = true;
    expect_same_at_every_count(c);
}

// spread_actuators() against spread_actuator() disc by disc, over steps whose
// discs move, change thrust and torque and stop swirling, into increments that
// start with -0 everywhere (a dropped +0 term would leave a -0).
TEST(AirflowThreads, KeptSpreadWeightsAddTheSameBitsAsTheSpread) {
    const GridShape s{24, 20, 22, 0.02f};
    const std::size_t sizes[3] = {std::size_t{s.nx + 1u} * s.ny * s.nz, std::size_t{s.nx} * (s.ny + 1u) * s.nz,
                                  std::size_t{s.nx} * s.ny * (s.nz + 1u)};
    std::array<std::vector<float>, 3> want;
    std::array<std::vector<float>, 3> got;
    SpreadCache cache;
    for (uint32_t step = 0; step < 7; ++step) {
        for (uint32_t c = 0; c < 3; ++c) {
            want[c].assign(sizes[c], -0.0f);
            got[c].assign(sizes[c], -0.0f);
        }
        const std::vector<ActuatorDisc> d = discs_at(s, step / 2u);  // a key held for two steps, then changed
        for (const ActuatorDisc& disc : d) spread_actuator(disc, s, 1.225f, 0.004f, want[0], want[1], want[2]);
        spread_actuators(d, s, 1.225f, 0.004f, got[0], got[1], got[2], cache);
        for (uint32_t c = 0; c < 3; ++c) EXPECT_TRUE(same(want[c], got[c])) << "step " << step << ", component " << c;
    }
}
