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
// Mutations (slice 1): one rhs shared by the threads (the open box goes red); both
// colours of a sweep in one pass, no barrier between them (the open box goes
// red); a kept +0 sum dropped (the spread test goes red). Physics' third, a
// same-colour wrap split across threads, cannot be built: make_grid()
// refuses an odd periodic axis.
//
// Slice 2 (the publish and the pressure levels on the pool), the same tests plus
// AThreadedPublish*, AVCycle* and TheSpreadCacheStaysWithinItsByteBound:
// mutations red: both colours of a smoothing sweep in one dispatch, the publish
// header merged in reverse plane order (the -0/+0 tie test), the cache bound
// halved. The channels are big enough (>= kThreadedLevelCells) that the finest
// level's smoothing and residual really go to the pool.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <utility>
#include <span>
#include <vector>

#include <glm/glm.hpp>

#include "airflow_solid_shapes.hpp"
#include "core/flow_volume.hpp"
#include "physics/airflow/actuator.hpp"
#include "physics/airflow/boundaries.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/multigrid.hpp"
#include "physics/airflow/pressure.hpp"
#include "physics/airflow/publish.hpp"

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
    c.shape = GridShape{32, 16, 20, 0.02f};
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
    c.shape = GridShape{20, 16, 32, 0.02f};
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

namespace {

[[nodiscard]] uint32_t bits_of(float x) { return std::bit_cast<uint32_t>(x); }

struct Published {
    spade::FlowVolume volume;
    PublishCounts counts;
    bool ok = false;
};

// The pools every threaded piece is held to: the serial pass is the reference.
struct Pools {
    WorkerPool two{2};
    WorkerPool three{3};
    WorkerPool eight{8};
    WorkerPool reversed{3, ItemOrder::reversed};
    [[nodiscard]] std::array<std::pair<const char*, WorkerPool*>, 4> all() {
        return {{{"2 threads", &two}, {"3 threads", &three}, {"8 threads", &eight}, {"items reversed", &reversed}}};
    }
};

// Noise, a few samples that publish as no data (a huge face, a NaN face), a block of
// solid cells with deep interior, and apertures with a closed plane.
[[nodiscard]] Published publish_noise(const GridShape& s, bool periodic_x, WorkerPool* pool) {
    Published out;
    DomainBc bc;
    if (periodic_x) {
        bc.face[kXMinus] = FaceBc::periodic;
        bc.face[kXPlus] = FaceBc::periodic;
    }
    spade::Result<MacGrid> grid = make_grid(s, bc);
    if (!grid) {
        ADD_FAILURE() << grid.error().context;
        return out;
    }
    MacGrid g = std::move(*grid);
    Lcg rng;
    for (std::vector<float>* a : {&g.u, &g.v, &g.w, &g.p}) {
        for (float& x : *a) x = 3.0f * rng.next();
    }
    g.u[g.u.size() / 3] = 1.0e6f;  // its two cells' centred u: past 65520, no data
    g.v[g.v.size() / 2] = std::numeric_limits<float>::quiet_NaN();
    const std::size_t cells = std::size_t{s.nx} * s.ny * s.nz;
    std::vector<uint8_t> solid(cells, 0);
    for (uint32_t k = 2; k < 8 && k < s.nz; ++k) {
        for (uint32_t j = 3; j < 9 && j < s.ny; ++j) {
            for (uint32_t i = 4; i < 12 && i < s.nx; ++i) solid[cell_index(s, i, j, k)] = 1;
        }
    }
    std::array<std::vector<float>, 3> theta;
    theta[0].assign(g.u.size(), 1.0f);
    theta[1].assign(g.v.size(), 1.0f);
    theta[2].assign(g.w.size(), 1.0f);
    for (uint32_t k = 0; k < s.nz; ++k) {
        for (uint32_t j = 0; j < s.ny; ++j) theta[0][u_index(s, s.nx / 2, j, k)] = 0.0f;  // a closed sheet
    }
    PublishInput in;
    in.origin = glm::vec3(1.0f, 2.0f, 3.0f);
    in.solid = solid;
    in.periodic = {periodic_x, false, false};
    in.theta = {theta[0], theta[1], theta[2]};
    spade::Result<PublishCounts> counts = publish_render_volume(g, in, out.volume, pool);
    if (!counts) {
        ADD_FAILURE() << counts.error().context;
        return out;
    }
    out.counts = *counts;
    out.ok = true;
    return out;
}

void expect_same_published(const Published& a, const Published& b, const char* what) {
    SCOPED_TRACE(what);
    ASSERT_TRUE(a.ok);
    ASSERT_TRUE(b.ok);
    const spade::FlowVolumeHeader& x = a.volume.header;
    const spade::FlowVolumeHeader& y = b.volume.header;
    EXPECT_EQ(x.nx, y.nx);
    EXPECT_EQ(x.ny, y.ny);
    EXPECT_EQ(x.nz, y.nz);
    EXPECT_EQ(bits_of(x.origin.x), bits_of(y.origin.x));
    EXPECT_EQ(bits_of(x.cell), bits_of(y.cell));
    for (uint32_t c = 0; c < spade::kFlowChannelCount; ++c) {
        EXPECT_EQ(bits_of(x.channel_min[c]), bits_of(y.channel_min[c])) << "min of channel " << c;
        EXPECT_EQ(bits_of(x.channel_max[c]), bits_of(y.channel_max[c])) << "max of channel " << c;
    }
    EXPECT_EQ(x.no_data, y.no_data);
    EXPECT_EQ(x.extrapolated_cells, y.extrapolated_cells);
    EXPECT_EQ(a.counts.extrapolated, b.counts.extrapolated);
    EXPECT_TRUE(a.volume.v == b.volume.v) << "v halves";
    EXPECT_TRUE(a.volume.s == b.volume.s) << "s halves";
}

void expect_noise_publish_same(const GridShape& s, bool periodic_x) {
    const Published serial = publish_noise(s, periodic_x, nullptr);
    ASSERT_TRUE(serial.ok);
    EXPECT_GT(serial.volume.header.no_data, 0u) << "the case must publish some no data";
    EXPECT_GT(serial.volume.header.extrapolated_cells, 0u) << "and some first-layer solid cells";
    Pools pools;
    for (const auto& [name, pool] : pools.all()) {
        expect_same_published(serial, publish_noise(s, periodic_x, pool), name);
    }
}

}  // namespace

// The publish on a pool: every cell's halves, the header's range, the no-data count and the
// first-layer count, against the serial pass, on an odd box and a box periodic in x.
TEST(AirflowThreads, AThreadedPublishIsTheSerialPublishOnAnOddBox) {
    expect_noise_publish_same(GridShape{19, 17, 15, 0.02f}, false);
}

TEST(AirflowThreads, AThreadedPublishIsTheSerialPublishOnAPeriodicBox) {
    expect_noise_publish_same(GridShape{16, 13, 14, 0.02f}, true);
}

// A tie of -0 and +0 in a channel's range keeps the first cell's: planes below the middle hold
// -0 in u, v, w and p, planes above +0, so a header merged in any other order than plane order
// ends with the wrong zero. (The serial header holds -0.)
TEST(AirflowThreads, AThreadedPublishKeepsTheFirstOfATieOfMinusAndPlusZero) {
    const GridShape s{8, 8, 8, 0.02f};
    const auto publish = [&](WorkerPool* pool) {
        Published out;
        spade::Result<MacGrid> grid = make_grid(s, DomainBc{});
        if (!grid) {
            ADD_FAILURE() << grid.error().context;
            return out;
        }
        MacGrid g = std::move(*grid);
        const auto fill = [&](std::vector<float>& a, std::size_t per_plane) {
            for (std::size_t q = 0; q < a.size(); ++q) a[q] = q / per_plane < s.nz / 2u ? -0.0f : 0.0f;
        };
        fill(g.u, std::size_t{s.nx + 1u} * s.ny);
        fill(g.v, std::size_t{s.nx} * (s.ny + 1u));
        fill(g.w, std::size_t{s.nx} * s.ny);
        fill(g.p, std::size_t{s.nx} * s.ny);
        spade::Result<PublishCounts> counts = publish_render_volume(g, PublishInput{}, out.volume, pool);
        if (!counts) {
            ADD_FAILURE() << counts.error().context;
            return out;
        }
        out.counts = *counts;
        out.ok = true;
        return out;
    };
    const Published serial = publish(nullptr);
    ASSERT_TRUE(serial.ok);
    for (const uint32_t c : {0u, 1u, 2u, 4u}) {
        EXPECT_EQ(bits_of(serial.volume.header.channel_min[c]), bits_of(-0.0f)) << "the serial min of channel " << c;
        EXPECT_EQ(bits_of(serial.volume.header.channel_max[c]), bits_of(-0.0f)) << "the serial max of channel " << c;
    }
    Pools pools;
    for (const auto& [name, pool] : pools.all()) expect_same_published(serial, publish(pool), name);
}

// One V-cycle and the residual on a pool: every level's p, b and r against the serial cycle, on
// an odd box and one periodic in x, both past kThreadedLevelCells at the finest level.
namespace {

void expect_v_cycle_same(uint32_t nx, uint32_t ny, uint32_t nz, bool periodic_x) {
    DomainBc bc;
    if (periodic_x) {
        bc.face[kXMinus] = FaceBc::periodic;
        bc.face[kXPlus] = FaceBc::periodic;
    }
    const auto make = [&] {
        Multigrid mg = make_multigrid(nx, ny, nz, {periodic_x, false, false});
        set_kinds(mg, face_kinds(bc));
        Lcg rng;
        for (float& x : mg.levels[0].b) x = rng.next();
        for (float& x : mg.levels[0].p) x = 0.5f * rng.next();
        return mg;
    };
    Multigrid serial = make();
    ASSERT_GE(serial.levels[0].p.size(), kThreadedLevelCells);
    v_cycle(serial);
    v_cycle(serial);
    residual(serial.levels[0]);
    Pools pools;
    for (const auto& [name, pool] : pools.all()) {
        SCOPED_TRACE(name);
        Multigrid on_pool = make();
        v_cycle(on_pool, pool);
        v_cycle(on_pool, pool);
        residual(on_pool.levels[0], pool);
        ASSERT_EQ(on_pool.levels.size(), serial.levels.size());
        for (std::size_t l = 0; l < serial.levels.size(); ++l) {
            EXPECT_TRUE(same(serial.levels[l].p, on_pool.levels[l].p)) << "p of level " << l;
            EXPECT_TRUE(same(serial.levels[l].b, on_pool.levels[l].b)) << "b of level " << l;
            EXPECT_TRUE(same(serial.levels[l].r, on_pool.levels[l].r)) << "r of level " << l;
        }
    }
}

}  // namespace

TEST(AirflowThreads, AVCycleOnAPoolIsTheSerialCycleOnAnOddBox) { expect_v_cycle_same(25, 21, 19, false); }

TEST(AirflowThreads, AVCycleOnAPoolIsTheSerialCycleOnAPeriodicBox) { expect_v_cycle_same(24, 20, 22, true); }

// The byte cap counts the kept weights (analysis::resolve() adds spread_cache_bytes() a rotor):
// the cache a real step builds is never larger than the bound, for discs of several sizes, in
// the middle of a grid, at its edge, and one that fills nearly all of a small one.
TEST(AirflowThreads, TheSpreadCacheStaysWithinItsByteBound) {
    struct Disc {
        GridShape shape;
        float radius_cells;
        glm::vec3 at;  // in cells
    };
    const Disc cases[] = {
        {GridShape{24, 20, 22, 0.02f}, 2.2f, glm::vec3(12.0f, 10.0f, 11.0f)},
        {GridShape{24, 20, 22, 0.02f}, 5.0f, glm::vec3(8.0f, 9.0f, 11.0f)},
        {GridShape{12, 12, 12, 0.05f}, 3.9f, glm::vec3(6.0f, 6.0f, 6.0f)},
        {GridShape{32, 32, 32, 0.004f}, 8.0f, glm::vec3(16.0f, 16.0f, 16.0f)},
        {GridShape{12, 40, 12, 0.01f}, 2.5f, glm::vec3(6.0f, 20.0f, 6.0f)},
    };
    for (const Disc& c : cases) {
        const GridShape& s = c.shape;
        std::array<std::vector<float>, 3> out;
        out[0].assign(std::size_t{s.nx + 1u} * s.ny * s.nz, 0.0f);
        out[1].assign(std::size_t{s.nx} * (s.ny + 1u) * s.nz, 0.0f);
        out[2].assign(std::size_t{s.nx} * s.ny * (s.nz + 1u), 0.0f);
        ActuatorDisc d;
        d.centre = c.at * s.dx;
        d.radius = c.radius_cells * s.dx;
        d.thrust = 1.0f;
        d.torque = 0.01f;
        const spade::Result<void> checked = check_disc(d, s);
        ASSERT_TRUE(checked.has_value()) << checked.error().context << " (radius " << c.radius_cells << " cells)";
        SpreadCache cache;
        const std::array<ActuatorDisc, 1> discs = {d};
        spread_actuators(discs, s, 1.225f, 0.004f, out[0], out[1], out[2], cache);
        const uint64_t kept = cache.bytes();
        const uint64_t bound = spread_cache_bytes(s, d.radius);
        EXPECT_GT(kept, 0u);
        EXPECT_LE(kept, bound) << s.nx << "x" << s.ny << "x" << s.nz << " radius " << c.radius_cells << " cells";
        EXPECT_GT(2u * bound, kept) << "the bound should not be loose by more than 2x for a disc well inside the grid";
    }
}
