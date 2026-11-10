// ---------------------------------------------------------------------------
// test_airflow_sgs.cpp -- the subgrid eddy viscosity (L52; engine/physics/airflow/sgs.hpp): Smagorinsky's
// nu_t = (Cs dx)^2 |S| added to the CPU step, FluidConfig::sgs_cs, 0 = off.
//
//   AirflowSgs  off touches nothing (no scratch, no health field); on changes the field; the analytic changes: a
//               constant shear's nu_t and the cap, a quadratic shear's d_y (nu_t d_y u), uniform flow and rigid
//               rotation left alone; the wall rule on a sphere (nu_t 0 in solid cells, solid samples unchanged, a
//               maximum principle with the cap and the wall limit engaged); the same bits at 1, 2, 3 and 8 threads
//               and on a reversed pool; the refusals by name; the reference quad's D1 box (solids) at 4 mm.
//   Opt-in, SPADE_SGS_FILM=1 (about 8 minutes in release; the quad film's own grid):
//               the film's wake at Cs 0.17 against Cs 0, ke_hi and the momentum flux under the discs.
//
// Mutations (slice 1): Cs read as cs instead of cs^2 (the analytic change goes red); nu_t not zeroed in solid cells
// (the wall test); the add done in place (the thread test); the cap off (the maximum principle).
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <complex>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../tools/flow_movie/cases.hpp"
#include "airflow_solid_shapes.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/sgs.hpp"
#include "physics/airflow/wall_load.hpp"
#include "scene/model_file.hpp"
#include "state/layout.hpp"
#include "vehicles/airflow_solids.hpp"
#include "vehicles/part_pose.hpp"

namespace {

using namespace spade::physics::airflow;

struct Lcg {
    uint32_t state = 2463534242u;
    float next() noexcept {  // [-1, 1)
        state = state * 1664525u + 1013904223u;
        return static_cast<float>(state >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }
};

const char* env_text(const char* name) {
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    return std::getenv(name);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
}

[[nodiscard]] MacGrid open_grid(const GridShape& s, const DomainBc& bc = DomainBc{}) {
    spade::Result<MacGrid> g = make_grid(s, bc);
    EXPECT_TRUE(g.has_value());
    return g ? std::move(*g) : MacGrid{};
}

void fill_noise(MacGrid& g, float amp, uint32_t seed = 2463534242u) {
    Lcg rng;
    rng.state = seed;
    for (std::vector<float>* a : {&g.u, &g.v, &g.w}) {
        for (float& x : *a) x = amp * rng.next();
    }
}

[[nodiscard]] bool same(const std::vector<float>& a, const std::vector<float>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
}

// One direct call of sgs_diffuse() on a grid with no solids.
SgsReport diffuse_once(MacGrid& g, float cs, float h, SgsScratch& s, const DomainBc& bc = DomainBc{}) {
    sgs_size(s, g.shape);
    return sgs_diffuse(g, bc, nullptr, nullptr, cs, h, s, nullptr);
}

// ---------------------------------------------------------------------------
// OFF, ON
// ---------------------------------------------------------------------------

TEST(AirflowSgs, TheSubgridViscosityIsOffByDefaultAndTouchesNothingOff) {
    EXPECT_EQ(FluidConfig{}.sgs_cs, 0.0f);
    const GridShape shape{25, 21, 19, 0.02f};
    const DomainBc bc;
    MacGrid g = open_grid(shape, bc);
    fill_noise(g, 0.5f);
    FluidConfig cfg{.rho = 1.225f, .nu = 30.0e-3f, .h = 0.004f};
    spade::Result<FluidScratch> s = make_fluid_scratch(g, bc, cfg);
    ASSERT_TRUE(s.has_value());
    for (int i = 0; i < 4; ++i) {
        const FluidHealth h = fluid_step(g, bc, cfg, *s);
        ASSERT_EQ(h.fault, FluidFault::none);
        EXPECT_EQ(h.sgs_capped_cells, 0u);
        EXPECT_EQ(h.sgs_limited_faces, 0u);
        EXPECT_EQ(h.sgs_nu_t_max, 0.0f);
    }
    EXPECT_TRUE(s->sgs.nut.empty()) << "off allocates nothing";
    EXPECT_TRUE(s->sgs.du.empty());
}

TEST(AirflowSgs, ASwitchedOnSubgridViscosityChangesTheFieldAndReportsItsNut) {
    const GridShape shape{25, 21, 19, 0.02f};
    const DomainBc bc;
    MacGrid a = open_grid(shape, bc);
    fill_noise(a, 0.5f);
    MacGrid b = a;
    FluidConfig off{.rho = 1.225f, .nu = 0.0f, .h = 0.004f};
    FluidConfig on = off;
    on.sgs_cs = 0.17f;
    auto sa = make_fluid_scratch(a, bc, off);
    auto sb = make_fluid_scratch(b, bc, on);
    ASSERT_TRUE(sa.has_value());
    ASSERT_TRUE(sb.has_value());
    EXPECT_FALSE(sb->sgs.nut.empty()) << "sized by make_fluid_scratch";
    FluidHealth hb;
    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(fluid_step(a, bc, off, *sa).fault, FluidFault::none);
        hb = fluid_step(b, bc, on, *sb);
        ASSERT_EQ(hb.fault, FluidFault::none);
    }
    EXPECT_FALSE(same(a.u, b.u));
    EXPECT_GT(hb.sgs_nu_t_max, 0.0f);
}

// ---------------------------------------------------------------------------
// THE ANALYTIC CHANGES
// ---------------------------------------------------------------------------

// u = gamma y, y = (j + 1/2) dx: |S| = gamma, nu_t = (Cs dx)^2 gamma everywhere (the edges' one-sided differences of a
// linear field are exact), the change 0 (nu_t is uniform and u is linear).
TEST(AirflowSgs, AConstantShearGivesTheAnalyticNutAndNoChange) {
    const GridShape shape{12, 12, 12, 0.02f};
    MacGrid g = open_grid(shape);
    constexpr float gamma = 20.0f;
    for (uint32_t k = 0; k < shape.nz; ++k) {
        for (uint32_t j = 0; j < shape.ny; ++j) {
            for (uint32_t i = 0; i <= shape.nx; ++i) {
                g.u[u_index(shape, i, j, k)] = gamma * (static_cast<float>(j) + 0.5f) * shape.dx;
            }
        }
    }
    const MacGrid before = g;
    SgsScratch s;
    const float cs = 0.17f;
    const SgsReport r = diffuse_once(g, cs, 1.0e-3f, s);
    const float want = cs * cs * shape.dx * shape.dx * gamma;
    EXPECT_NEAR(r.nu_t_max, want, 1e-4f * want);
    EXPECT_EQ(r.capped_cells, 0u);
    EXPECT_EQ(r.limited_faces, 0u);
    // The faces with both y edges present: at a domain edge the zero-gradient ghost has no term (viscous.cpp's).
    for (uint32_t k = 0; k < shape.nz; ++k) {
        for (uint32_t j = 1; j + 1 < shape.ny; ++j) {
            for (uint32_t i = 1; i < shape.nx; ++i) {
                const std::size_t f = u_index(shape, i, j, k);
                EXPECT_NEAR(g.u[f], before.u[f], 1e-5f * std::fabs(before.u[f]) + 1e-6f);
            }
        }
    }
}

// Over the cap: every cell held at dx^2 / (8 h), counted, every face finite and inside the field's range.
TEST(AirflowSgs, ANutAboveTheCapIsHeldAndCounted) {
    const GridShape shape{12, 12, 12, 0.02f};
    MacGrid g = open_grid(shape);
    constexpr float gamma = 1.0e3f;
    for (uint32_t k = 0; k < shape.nz; ++k) {
        for (uint32_t j = 0; j < shape.ny; ++j) {
            for (uint32_t i = 0; i <= shape.nx; ++i) {
                g.u[u_index(shape, i, j, k)] = gamma * (static_cast<float>(j) + 0.5f) * shape.dx;
            }
        }
    }
    const float lo = *std::min_element(g.u.begin(), g.u.end());
    const float hi = *std::max_element(g.u.begin(), g.u.end());
    SgsScratch s;
    const float h = 0.01f;
    const SgsReport r = diffuse_once(g, 0.17f, h, s);
    const float cap = shape.dx * shape.dx / (8.0f * h);
    EXPECT_EQ(r.capped_cells, shape.nx * shape.ny * shape.nz);
    EXPECT_GT(r.nu_t_max, cap);
    for (const float nut : s.nut) EXPECT_FLOAT_EQ(nut, cap);
    for (const float x : g.u) {
        ASSERT_TRUE(std::isfinite(x));
        EXPECT_GE(x, lo - 1e-6f);
        EXPECT_LE(x, hi + 1e-6f);
    }
}

// u = a y^2: S_xy = a y, |S| = 2 a y, nu_t = 2 c2 a y (c2 = (Cs dx)^2), d_y (nu_t d_y u) = 8 c2 a^2 y -- exactly, on
// the grid (the central differences of a quadratic are exact), at faces two cells in from every edge in y.
TEST(AirflowSgs, AQuadraticShearGivesTheAnalyticDiffusion) {
    const GridShape shape{14, 14, 10, 0.02f};
    MacGrid g = open_grid(shape);
    constexpr float a = 200.0f;
    for (uint32_t k = 0; k < shape.nz; ++k) {
        for (uint32_t j = 0; j < shape.ny; ++j) {
            const float y = (static_cast<float>(j) + 0.5f) * shape.dx;
            for (uint32_t i = 0; i <= shape.nx; ++i) g.u[u_index(shape, i, j, k)] = a * y * y;
        }
    }
    const MacGrid before = g;
    SgsScratch s;
    const float cs = 0.17f;
    const float h = 5.0e-3f;
    const SgsReport r = diffuse_once(g, cs, h, s);
    ASSERT_EQ(r.capped_cells, 0u);
    const float c2 = cs * cs * shape.dx * shape.dx;
    uint32_t checked = 0;
    for (uint32_t k = 1; k + 1 < shape.nz; ++k) {
        for (uint32_t j = 2; j + 2 < shape.ny; ++j) {
            for (uint32_t i = 1; i < shape.nx; ++i) {
                const float y = (static_cast<float>(j) + 0.5f) * shape.dx;
                const float want = h * 8.0f * c2 * a * a * y;
                const std::size_t f = u_index(shape, i, j, k);
                EXPECT_NEAR(g.u[f] - before.u[f], want, 5e-3f * want) << i << ' ' << j << ' ' << k;
                ++checked;
            }
        }
    }
    EXPECT_GT(checked, 0u);
}

// Smagorinsky must not act on a flow with no strain: a uniform flow and a rigid rotation.
TEST(AirflowSgs, UniformFlowAndRigidRotationAreLeftAlone) {
    const GridShape shape{16, 16, 16, 0.02f};
    SgsScratch s;
    {
        MacGrid g = open_grid(shape);
        std::fill(g.u.begin(), g.u.end(), 3.0f);
        const MacGrid before = g;
        const SgsReport r = diffuse_once(g, 0.17f, 1.0e-3f, s);
        EXPECT_EQ(r.nu_t_max, 0.0f);
        EXPECT_TRUE(same(g.u, before.u));
    }
    {
        MacGrid g = open_grid(shape);
        constexpr float omega = 50.0f;  // rad/s about the grid's centre line, along z
        const float mid = 0.5f * static_cast<float>(shape.nx) * shape.dx;
        for (uint32_t k = 0; k < shape.nz; ++k) {
            for (uint32_t j = 0; j < shape.ny; ++j) {
                for (uint32_t i = 0; i <= shape.nx; ++i) {
                    g.u[u_index(shape, i, j, k)] = -omega * ((static_cast<float>(j) + 0.5f) * shape.dx - mid);
                }
            }
        }
        for (uint32_t k = 0; k < shape.nz; ++k) {
            for (uint32_t j = 0; j <= shape.ny; ++j) {
                for (uint32_t i = 0; i < shape.nx; ++i) {
                    g.v[v_index(shape, i, j, k)] = omega * ((static_cast<float>(i) + 0.5f) * shape.dx - mid);
                }
            }
        }
        const MacGrid before = g;
        const SgsReport r = diffuse_once(g, 0.17f, 1.0e-3f, s);
        EXPECT_LT(r.nu_t_max, 1e-7f);
        for (std::size_t f = 0; f < g.u.size(); ++f) EXPECT_NEAR(g.u[f], before.u[f], 1e-6f);
        for (std::size_t f = 0; f < g.v.size(); ++f) EXPECT_NEAR(g.v[f], before.v[f], 1e-6f);
    }
}

// ---------------------------------------------------------------------------
// THE WALL RULE
// ---------------------------------------------------------------------------

struct SphereBox {
    GridShape shape{22, 16, 18, 0.02f};
    DomainBc bc;
    MacGrid g;
    FluidScratch s;
};

[[nodiscard]] bool sphere_box(SphereBox& b, const FluidConfig& cfg) {
    b.g = open_grid(b.shape, b.bc);
    auto made = make_fluid_scratch(b.g, b.bc, cfg);
    if (!made) return false;
    b.s = std::move(*made);
    airflow_test::Scene scene;
    const float dx = b.shape.dx;
    scene.shapes.push_back(airflow_test::Shape{.name = "sphere",
                                               .kind = airflow_test::Kind::sphere,
                                               .centre = glm::vec3(11.0f * dx, 7.0f * dx, 9.0f * dx),
                                               .radius = 3.5f * dx,
                                               .min_feature = 7.0f * dx});
    return set_solids(b.s, b.g, b.bc, scene.parts(), scene.sampler()).has_value();
}

// A uniform wind past a still sphere: nu_t 0 in the solid cells, the solid's samples unchanged, the faces beside it
// pulled toward the wall (the wall link), and no face leaves [0, U].
TEST(AirflowSgs, TheWallRuleHoldsNutAtZeroInSolidCellsAndPullsTheFaceBesideAWallToIt) {
    SphereBox b;
    FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.002f, .interior = SolidInterior::rigid};
    ASSERT_TRUE(sphere_box(b, cfg));
    constexpr float wind = 5.0f;
    std::fill(b.g.u.begin(), b.g.u.end(), wind);
    apply_solid_velocities(b.g, b.s.solids, SolidInterior::rigid);
    const MacGrid before = b.g;
    SgsScratch sg;
    sgs_size(sg, b.shape);
    const SgsReport r = sgs_diffuse(b.g, b.bc, &b.s.solids, b.s.regions.solid.data(), 0.17f, cfg.h, sg, nullptr);
    uint32_t solid_cells = 0;
    for (std::size_t c = 0; c < b.s.regions.solid.size(); ++c) {
        if (b.s.regions.solid[c] == 0u) continue;
        ++solid_cells;
        EXPECT_EQ(sg.nut[c], 0.0f) << "solid cell " << c;
    }
    EXPECT_GT(solid_cells, 0u);
    EXPECT_GT(r.nu_t_max, 0.0f);
    uint32_t pulled = 0;
    for (std::size_t f = 0; f < b.g.u.size(); ++f) {
        const bool held = closed(b.s.solids.theta[0][f]) || b.s.solids.phi[0][f] < 0.0f;
        if (held) {
            EXPECT_EQ(b.g.u[f], before.u[f]) << "held sample " << f;
        }
        EXPECT_GE(b.g.u[f], 0.0f - 1e-6f);
        EXPECT_LE(b.g.u[f], wind + 1e-6f);
        if (!held && b.g.u[f] < before.u[f] - 1e-6f) ++pulled;
    }
    EXPECT_GT(pulled, 0u);
}

// Random noise, Cs at its largest, h long: nu_t sits at the cap nearly everywhere and wall links are weighted up to
// 1 / f. The explicit step with its limit is a weighted mean of neighbours and wall values (0): no face leaves the
// noise's range, and the cap and the limit were both met.
TEST(AirflowSgs, TheCapAndTheWallLimitKeepEveryFaceInsideTheFieldsRange) {
    SphereBox b;
    FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.05f, .interior = SolidInterior::rigid};
    ASSERT_TRUE(sphere_box(b, cfg));
    fill_noise(b.g, 1.0f);
    apply_solid_velocities(b.g, b.s.solids, SolidInterior::rigid);
    SgsScratch sg;
    sgs_size(sg, b.shape);
    uint32_t capped = 0;
    uint32_t limited = 0;
    for (int step = 0; step < 4; ++step) {
        const SgsReport r =
            sgs_diffuse(b.g, b.bc, &b.s.solids, b.s.regions.solid.data(), kSgsCsMax, cfg.h, sg, nullptr);
        capped += r.capped_cells;
        limited += r.limited_faces;
        for (const std::vector<float>* a : {&b.g.u, &b.g.v, &b.g.w}) {
            for (const float x : *a) {
                ASSERT_TRUE(std::isfinite(x));
                ASSERT_LE(std::fabs(x), 1.0f + 1e-5f) << "step " << step;
            }
        }
    }
    EXPECT_GT(capped, 0u);
    EXPECT_GT(limited, 0u) << "the wall limit was never engaged: the test does not reach it";
}

// ---------------------------------------------------------------------------
// THREADS: the same bits at every count
// ---------------------------------------------------------------------------

struct SgsRun {
    MacGrid g;
    std::vector<FluidHealth> health;
};

[[nodiscard]] SgsRun threaded_run(bool sphere, uint32_t threads, bool reversed) {
    SgsRun out;
    const GridShape shape{22, 16, 18, 0.02f};
    DomainBc bc;
    bc.face.fill(FaceBc::wall);
    bc.face[kYPlus] = FaceBc::open;
    if (!sphere) bc = DomainBc{};
    out.g = open_grid(shape, bc);
    fill_noise(out.g, 0.5f);
    FluidConfig cfg{.rho = 1.225f, .nu = 1.5e-5f * 2000.0f, .h = 0.004f};
    cfg.threads = threads;
    cfg.sgs_cs = 0.17f;
    auto s = make_fluid_scratch(out.g, bc, cfg);
    if (!s) {
        ADD_FAILURE() << s.error().context;
        return out;
    }
    if (reversed) s->pool = std::make_shared<WorkerPool>(threads, ItemOrder::reversed);
    if (sphere) {
        airflow_test::Scene scene;
        const float dx = shape.dx;
        scene.shapes.push_back(airflow_test::Shape{.name = "sphere",
                                                   .kind = airflow_test::Kind::sphere,
                                                   .centre = glm::vec3(11.0f * dx, 7.0f * dx, 9.0f * dx),
                                                   .radius = 3.5f * dx,
                                                   .min_feature = 7.0f * dx});
        EXPECT_TRUE(set_solids(*s, out.g, bc, scene.parts(), scene.sampler()).has_value());
    }
    for (uint32_t step = 0; step < 5; ++step) out.health.push_back(fluid_step(out.g, bc, cfg, *s));
    return out;
}

void expect_same_runs(const SgsRun& one, const SgsRun& many, const char* what) {
    SCOPED_TRACE(what);
    EXPECT_TRUE(same(one.g.u, many.g.u)) << "u";
    EXPECT_TRUE(same(one.g.v, many.g.v)) << "v";
    EXPECT_TRUE(same(one.g.w, many.g.w)) << "w";
    EXPECT_TRUE(same(one.g.p, many.g.p)) << "p";
    ASSERT_EQ(one.health.size(), many.health.size());
    for (std::size_t i = 0; i < one.health.size(); ++i) {
        EXPECT_EQ(one.health[i].fault, many.health[i].fault);
        EXPECT_EQ(one.health[i].sgs_capped_cells, many.health[i].sgs_capped_cells) << "step " << i;
        EXPECT_EQ(one.health[i].sgs_limited_faces, many.health[i].sgs_limited_faces) << "step " << i;
        EXPECT_EQ(std::bit_cast<uint32_t>(one.health[i].sgs_nu_t_max),
                  std::bit_cast<uint32_t>(many.health[i].sgs_nu_t_max))
            << "step " << i;
    }
}

TEST(AirflowSgs, TheStepIsTheSameBitsAtEveryThreadCountWithAndWithoutSolids) {
    for (const bool sphere : {false, true}) {
        SCOPED_TRACE(sphere ? "walled box with a sphere" : "open box");
        const SgsRun one = threaded_run(sphere, 1u, false);
        ASSERT_EQ(one.health.size(), 5u);
        EXPECT_GT(one.health.back().sgs_nu_t_max, 0.0f);
        expect_same_runs(one, threaded_run(sphere, 2u, false), "2 threads");
        expect_same_runs(one, threaded_run(sphere, 3u, false), "3 threads");
        expect_same_runs(one, threaded_run(sphere, 8u, false), "8 threads");
        expect_same_runs(one, threaded_run(sphere, 3u, true), "items reversed");
    }
}

// ---------------------------------------------------------------------------
// REFUSALS BY NAME
// ---------------------------------------------------------------------------

TEST(AirflowSgs, ABadCsIsRefusedByNameAtTheScratchAndFaultsTheStep) {
    const GridShape shape{12, 12, 12, 0.02f};
    const DomainBc bc;
    MacGrid g = open_grid(shape, bc);
    for (const float bad : {-0.1f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
                            kSgsCsMax * 1.001f}) {
        FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.004f, .sgs_cs = bad};
        auto s = make_fluid_scratch(g, bc, cfg);
        ASSERT_FALSE(s.has_value()) << bad;
        EXPECT_NE(s.error().context.find("sgs_cs"), std::string::npos) << s.error().context;
    }
    FluidConfig ok{.rho = 1.225f, .nu = 0.0f, .h = 0.004f, .sgs_cs = kSgsCsMax};
    auto s = make_fluid_scratch(g, bc, ok);
    ASSERT_TRUE(s.has_value());
    // A Cs changed after the scratch was made is caught by the step, named, and kept (L6).
    ok.sgs_cs = -1.0f;
    const FluidHealth h = fluid_step(g, bc, ok, *s);
    EXPECT_EQ(h.fault, FluidFault::bad_sgs_cs);
    EXPECT_NE(describe(h, shape).find("sgs_cs"), std::string::npos) << describe(h, shape);
    ok.sgs_cs = 0.1f;
    EXPECT_EQ(fluid_step(g, bc, ok, *s).fault, FluidFault::bad_sgs_cs) << "the fault is kept until a new scratch";
}

// ---------------------------------------------------------------------------
// D1 WITH SOLIDS (RED 8): the reference quad's airframe at 4 mm, in a 3 m/s wind, at Cs 0 and Cs 0.17.
// ---------------------------------------------------------------------------

struct D1Out {
    std::string error;
    bool finite = true;
    uint32_t held_faces = 0;
    float held_max_dev = 0.0f;  // the largest |u - wall| on a held (closed) face
    glm::vec3 force{0.0f};      // the parts' load at the last step
    FluidHealth last{};
    uint32_t solid_cells = 0;
};

[[nodiscard]] D1Out d1_run(float cs, uint32_t steps) {
    D1Out out;
    constexpr float kDx = 0.004f;
    constexpr float kWind = 3.0f;
    const std::filesystem::path file =
        std::filesystem::path(SPADE_ASSETS_DIR) / "library" / "drones" / "reference_quad.model.yaml";
    auto loaded = spade::scene::load_model_file(file);
    if (!loaded) {
        out.error = "the library quad: " + loaded.error().context;
        return out;
    }
    const GridShape shape{96, 32, 96, kDx};
    const glm::vec3 origin = -0.5f * kDx * glm::vec3(96.0f, 32.0f, 96.0f);
    DomainBc bc;
    bc.ambient = glm::vec3(kWind, 0.0f, 0.0f);
    FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.5f * kDx / kWind, .n_v = kSolidsCgIterations,
                    .interior = SolidInterior::extrapolate};
    cfg.sgs_cs = cs;
    spade::BodyState body{};
    body.pos = glm::vec3(0.0f);
    body.orient = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    const std::array<float, 4> still{};
    auto posed =
        spade::vehicles::posed_parts(loaded->model, body, still, still, spade::vehicles::PartFilter::rigid_only);
    if (!posed) {
        out.error = "posed_parts: " + posed.error().context;
        return out;
    }
    MacGrid g = open_grid(shape, bc);
    std::fill(g.u.begin(), g.u.end(), kWind);
    auto scratch = make_fluid_scratch(g, bc, cfg);
    if (!scratch) {
        out.error = "make_fluid_scratch: " + scratch.error().context;
        return out;
    }
    FluidScratch& s = *scratch;
    const spade::vehicles::AirflowSolids solids(loaded->model, *posed, origin);
    auto parts = solids.parts();
    if (!parts) {
        out.error = "parts: " + parts.error().context;
        return out;
    }
    if (auto set = set_solids(s, g, bc, *parts, solids.sampler()); !set) {
        out.error = "set_solids: " + set.error().context;
        return out;
    }
    for (uint32_t i = 0; i < steps; ++i) {
        out.last = fluid_step(g, bc, cfg, s);
        if (out.last.fault != FluidFault::none) {
            out.error = "fluid_step " + std::to_string(i) + " faulted: " + describe(out.last, shape);
            return out;
        }
    }
    for (const std::vector<float>* a : {&g.u, &g.v, &g.w, &g.p}) {
        for (const float x : *a) out.finite = out.finite && std::isfinite(x);
    }
    const std::array<const std::vector<float>*, 3> comp = {&g.u, &g.v, &g.w};
    for (uint32_t c = 0; c < 3; ++c) {
        for (std::size_t f = 0; f < comp[c]->size(); ++f) {
            if (!closed(s.solids.theta[c][f])) continue;
            ++out.held_faces;
            out.held_max_dev = std::max(out.held_max_dev, std::fabs((*comp[c])[f] - s.solids.wall[c][f]));
        }
    }
    for (const uint8_t m : s.regions.solid) out.solid_cells += m;
    out.force = wall_load(g, s.solids, s.regions, glm::vec3(0.0f)).force;
    return out;
}

// A sphere in a 5 m/s wind, 40 steps at Cs 0.17: finite, no air through a closed face, nu_t reported.
TEST(AirflowSgs, ASphereInAWindAtCs017StaysFiniteWithNoFluxIntoClosedFaces) {
    SphereBox b;
    b.bc.ambient = glm::vec3(5.0f, 0.0f, 0.0f);
    FluidConfig cfg{.rho = 1.225f, .nu = 0.0f, .h = 0.002f, .n_v = kSolidsCgIterations,
                    .interior = SolidInterior::rigid, .sgs_cs = 0.17f};
    ASSERT_TRUE(sphere_box(b, cfg));
    std::fill(b.g.u.begin(), b.g.u.end(), 5.0f);
    FluidHealth h;
    for (int step = 0; step < 40; ++step) {
        h = fluid_step(b.g, b.bc, cfg, b.s);
        ASSERT_EQ(h.fault, FluidFault::none) << describe(h, b.shape);
    }
    EXPECT_GT(h.sgs_nu_t_max, 0.0f);
    uint32_t held = 0;
    const std::array<const std::vector<float>*, 3> comp = {&b.g.u, &b.g.v, &b.g.w};
    for (uint32_t c = 0; c < 3; ++c) {
        for (std::size_t f = 0; f < comp[c]->size(); ++f) {
            ASSERT_TRUE(std::isfinite((*comp[c])[f]));
            if (!closed(b.s.solids.theta[c][f])) continue;
            ++held;
            EXPECT_LE(std::fabs((*comp[c])[f] - b.s.solids.wall[c][f]), 1e-6f) << "a closed face moved";
        }
    }
    EXPECT_GT(held, 0u);
}

// 40 steps at Cs 0.17 with the airframe's solids stay finite and move no air through a closed face; the parts' load
// is stated at Cs 0 and Cs 0.17 (report only: no band).
TEST(AirflowSgs, TheReferenceQuadsAirframeAtCs017StaysFiniteWithNoFluxIntoClosedFaces) {
    if (env_text("SPADE_SGS_D1") == nullptr) GTEST_SKIP() << "set SPADE_SGS_D1=1 (about 1 minute in release)";
    const D1Out on = d1_run(0.17f, 40u);
    ASSERT_TRUE(on.error.empty()) << on.error;
    EXPECT_TRUE(on.finite);
    EXPECT_GT(on.solid_cells, 0u);
    EXPECT_GT(on.held_faces, 0u);
    EXPECT_LE(on.held_max_dev, 1e-6f) << "a closed face moved: air through a solid";
    EXPECT_GT(on.last.sgs_nu_t_max, 0.0f);
    const D1Out off = d1_run(0.0f, 40u);
    ASSERT_TRUE(off.error.empty()) << off.error;
    std::printf("[ sgs     ] D1 airframe at 4 mm, 3 m/s wind, 40 steps: the parts' load (N, grid axes, x downwind) at "
                "Cs 0: (%.5f, %.5f, %.5f); at Cs 0.17: (%.5f, %.5f, %.5f); nu_t max %.3g m2/s, capped cells %u, "
                "wall-limited faces %u\n",
                static_cast<double>(off.force.x), static_cast<double>(off.force.y), static_cast<double>(off.force.z),
                static_cast<double>(on.force.x), static_cast<double>(on.force.y), static_cast<double>(on.force.z),
                static_cast<double>(on.last.sgs_nu_t_max), on.last.sgs_capped_cells, on.last.sgs_limited_faces);
}

// ---------------------------------------------------------------------------
// THE QUAD FILM'S WAKE, opt-in (SPADE_SGS_FILM=1; about 8 minutes in release): 900 steps (0.45 s) of
// flow_movie's quad case at Cs 0 and at Cs 0.17, the same box and the same discs.
//   RED 2  ke_hi at Cs 0.17 is below half of ke_hi at Cs 0. ke_hi = the share of the kinetic energy at wavenumbers
//          above pi / (2 dx) in the box +-0.28 m across the discs' centre, from 0.20 m above the bottom face to
//          0.12 m under the hubs (the cell-centre velocity, Hann-windowed on each axis); the mean over steps 700,
//          750, 800, 850 and 900. The half line is the lead's, fixed before the first run.
//   RED 3  R = the mean over the same five steps of the momentum flux rho sum(v^2) dA through the horizontal plane
//          0.30 m under the hubs, over the rotors' total thrust 4T = 5.158 N: inside Test/Docs' band, at Cs 0 and
//          at Cs 0.17 alike. A single step is not tested (the 1.2 s probe run's single step read +40%).
// The bands are Test/Docs'; the run prints every number it measured beside them.
// ---------------------------------------------------------------------------

// RED 3's band on R (Test/Docs, docs/design/test-docs/2026-11-10-sgs-film-bands.md): centre 1 by momentum theory,
// edge 0.10 = the measured offset 0.014 + 3 x the dump sd 0.025, rounded up. Calibrated on this box and window:
// R 1.009 at Cs 0 and 1.004 at Cs 0.17, ke_hi means 0.0711 and 0.0102 (Physics, 2026-11-10). A different
// configuration or window needs a new calibration, not a wider band.
constexpr double kFluxRatioLo = 0.90;
constexpr double kFluxRatioHi = 1.10;

// The share of the kinetic energy of a block of cell-centre velocities (z, y, x, 3) above k = pi / (2 dx): Hann
// window on each axis, a direct DFT per axis (the block is about 56 x 64 x 56).
[[nodiscard]] double ke_share_above(const std::vector<float>& vel, uint32_t nz, uint32_t ny, uint32_t nx, double dx) {
    const std::size_t total = std::size_t{nz} * ny * nx;
    const auto hann = [](uint32_t n, uint32_t i) {
        return 0.5 - 0.5 * std::cos(2.0 * std::numbers::pi * i / static_cast<double>(n - 1u));
    };
    const auto freq = [&](uint32_t n, uint32_t m) {  // numpy's fftfreq, times 2 pi
        const int64_t mm =
            m < (n + 1u) / 2u ? static_cast<int64_t>(m) : static_cast<int64_t>(m) - static_cast<int64_t>(n);
        return 2.0 * std::numbers::pi * static_cast<double>(mm) / (static_cast<double>(n) * dx);
    };
    const auto table = [](uint32_t n) {
        std::vector<std::complex<double>> t(std::size_t{n} * n);
        for (uint32_t m = 0; m < n; ++m) {
            for (uint32_t i = 0; i < n; ++i) {
                const double a = -2.0 * std::numbers::pi * static_cast<double>(m) * i / static_cast<double>(n);
                t[std::size_t{m} * n + i] = {std::cos(a), std::sin(a)};
            }
        }
        return t;
    };
    const std::array<std::vector<std::complex<double>>, 3> tab = {table(nz), table(ny), table(nx)};
    const uint32_t dims[3] = {nz, ny, nx};
    std::vector<double> energy(total, 0.0);
    std::vector<std::complex<double>> a(total);
    std::vector<std::complex<double>> b(total);
    for (int c = 0; c < 3; ++c) {
        for (uint32_t z = 0; z < nz; ++z) {
            for (uint32_t y = 0; y < ny; ++y) {
                for (uint32_t x = 0; x < nx; ++x) {
                    const std::size_t at = (std::size_t{z} * ny + y) * nx + x;
                    a[at] = static_cast<double>(vel[at * 3u + static_cast<std::size_t>(c)]) * hann(nz, z) *
                            hann(ny, y) * hann(nx, x);
                }
            }
        }
        for (int axis = 0; axis < 3; ++axis) {
            const uint32_t n = dims[axis];
            const std::size_t stride = axis == 0 ? std::size_t{ny} * nx : (axis == 1 ? std::size_t{nx} : 1u);
            for (std::size_t base = 0; base < total; ++base) {
                if ((base / stride) % n != 0u) continue;  // the first sample of each line
                for (uint32_t m = 0; m < n; ++m) {
                    std::complex<double> acc(0.0, 0.0);
                    for (uint32_t i = 0; i < n; ++i) acc += tab[axis][std::size_t{m} * n + i] * a[base + i * stride];
                    b[base + m * stride] = acc;
                }
            }
            a.swap(b);
        }
        for (std::size_t at = 0; at < total; ++at) energy[at] += std::norm(a[at]);
    }
    double sum = 0.0;
    double high = 0.0;
    const double k_cut = std::numbers::pi / (2.0 * dx);
    for (uint32_t z = 0; z < nz; ++z) {
        const double kz = freq(nz, z);
        for (uint32_t y = 0; y < ny; ++y) {
            const double ky = freq(ny, y);
            for (uint32_t x = 0; x < nx; ++x) {
                const double kx = freq(nx, x);
                const std::size_t at = (std::size_t{z} * ny + y) * nx + x;
                sum += energy[at];
                if (std::sqrt(kx * kx + ky * ky + kz * kz) > k_cut) high += energy[at];
            }
        }
    }
    return high / sum;
}

struct FilmStats {
    std::vector<double> ke;     // at steps 700, 750, ... 900
    std::vector<double> ratio;  // flux / 4T at the same steps
    double ke_mean = 0.0;
    double ratio_mean = 0.0;
    std::string error;
};

[[nodiscard]] FilmStats film_run(float cs) {
    FilmStats out;
    spade::Result<flow_movie::AirflowCase> made = flow_movie::quad_hover_case();
    if (!made) {
        out.error = made.error().context;
        return out;
    }
    const flow_movie::AirflowCase& c = *made;
    MacGrid g = open_grid(c.shape, c.bc);
    FluidConfig cfg = c.fluid;
    cfg.sgs_cs = cs;
    auto scratch = make_fluid_scratch(g, c.bc, cfg);
    if (!scratch) {
        out.error = scratch.error().context;
        return out;
    }
    glm::vec3 mid(0.0f);
    for (const ActuatorDisc& d : c.discs) mid += d.centre;
    mid /= static_cast<float>(c.discs.size());
    const double dx = c.shape.dx;
    const double total_thrust = 4.0 * c.discs[0].thrust;
    const auto cells = [&](double m) { return static_cast<uint32_t>(std::lround(m / dx)); };
    const uint32_t x0 = cells(mid.x - 0.28);
    const uint32_t x1 = cells(mid.x + 0.28);
    const uint32_t z0 = cells(mid.z - 0.28);
    const uint32_t z1 = cells(mid.z + 0.28);
    const uint32_t y0 = cells(0.20);
    const uint32_t y1 = cells(mid.y - 0.12);
    const uint32_t jp = cells(mid.y - 0.30);
    for (uint32_t step = 1; step <= 900u; ++step) {
        for (const ActuatorDisc& d : c.discs) {
            spread_actuator(d, c.shape, cfg.rho, cfg.h, scratch->du, scratch->dv, scratch->dw);
        }
        const FluidHealth h = fluid_step(g, c.bc, cfg, *scratch);
        if (h.fault != FluidFault::none) {
            out.error = "fluid_step " + std::to_string(step) + ": " + describe(h, c.shape);
            return out;
        }
        if (step < 700u || step % 50u != 0u) continue;
        std::vector<float> vel;
        vel.reserve(std::size_t{z1 - z0} * (y1 - y0) * (x1 - x0) * 3u);
        for (uint32_t k = z0; k < z1; ++k) {
            for (uint32_t j = y0; j < y1; ++j) {
                for (uint32_t i = x0; i < x1; ++i) {
                    vel.push_back(0.5f * (g.u[u_index(c.shape, i, j, k)] + g.u[u_index(c.shape, i + 1u, j, k)]));
                    vel.push_back(0.5f * (g.v[v_index(c.shape, i, j, k)] + g.v[v_index(c.shape, i, j + 1u, k)]));
                    vel.push_back(0.5f * (g.w[w_index(c.shape, i, j, k)] + g.w[w_index(c.shape, i, j, k + 1u)]));
                }
            }
        }
        out.ke.push_back(ke_share_above(vel, z1 - z0, y1 - y0, x1 - x0, dx));
        double sum_v2 = 0.0;
        for (uint32_t k = 0; k < c.shape.nz; ++k) {
            for (uint32_t i = 0; i < c.shape.nx; ++i) {
                const double v = 0.5 * (static_cast<double>(g.v[v_index(c.shape, i, jp, k)]) +
                                        static_cast<double>(g.v[v_index(c.shape, i, jp + 1u, k)]));
                sum_v2 += v * v;
            }
        }
        out.ratio.push_back(static_cast<double>(cfg.rho) * sum_v2 * dx * dx / total_thrust);
    }
    for (const double k : out.ke) out.ke_mean += k / static_cast<double>(out.ke.size());
    for (const double r : out.ratio) out.ratio_mean += r / static_cast<double>(out.ratio.size());
    return out;
}

TEST(AirflowSgsFilm, TheQuadFilmsWakeLosesItsGridScaleEnergyAndKeepsItsMomentum) {
    if (env_text("SPADE_SGS_FILM") == nullptr) {
        GTEST_SKIP() << "set SPADE_SGS_FILM=1 to run the quad film's 0.45 s twice";
    }
    const auto t0 = std::chrono::steady_clock::now();
    const FilmStats off = film_run(0.0f);
    ASSERT_TRUE(off.error.empty()) << off.error;
    const FilmStats on = film_run(0.17f);
    ASSERT_TRUE(on.error.empty()) << on.error;
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("[ sgs     ] quad film, 900 steps at Cs 0 and Cs 0.17 (%.0f s, %u thread(s); calibration:"
                " R 1.009 / 1.004, ke_hi 0.0711 / 0.0102):\n",
                secs, default_fluid_threads());
    for (std::size_t i = 0; i < off.ke.size(); ++i) {
        std::printf("[ sgs     ]   step %zu: ke_hi %.4f / %.4f, flux/4T %.3f / %.3f (Cs 0 / Cs 0.17)\n", 700 + 50 * i,
                    off.ke[i], on.ke[i], off.ratio[i], on.ratio[i]);
    }
    std::printf("[ sgs     ] five-step means: ke_hi %.4f / %.4f (ratio %.3f), flux/4T %.3f / %.3f; band %.2f..%.2f\n",
                off.ke_mean, on.ke_mean, on.ke_mean / off.ke_mean, off.ratio_mean, on.ratio_mean, kFluxRatioLo,
                kFluxRatioHi);
    ASSERT_TRUE(std::isfinite(off.ke_mean) && std::isfinite(on.ke_mean) && std::isfinite(off.ratio_mean) &&
                std::isfinite(on.ratio_mean))
        << "a non-finite mean cannot pass";
    ASSERT_GT(off.ke_mean, 0.0) << "a Cs 0 mean of zero cannot pass below half";
    EXPECT_LT(on.ke_mean, 0.5 * off.ke_mean) << "RED 2";
    EXPECT_GE(off.ratio_mean, kFluxRatioLo) << "RED 3, Cs 0";
    EXPECT_LE(off.ratio_mean, kFluxRatioHi) << "RED 3, Cs 0";
    EXPECT_GE(on.ratio_mean, kFluxRatioLo) << "RED 3, Cs 0.17";
    EXPECT_LE(on.ratio_mean, kFluxRatioHi) << "RED 3, Cs 0.17";
}

}  // namespace
