// Manual-only benchmarks for the airflow solver core's CPU reference (the
// airflow CPU-performance task; the lead, 2026-10-12): one family per stage
// of fluid_step -- advection, viscosity, projection -- plus the render-volume
// publish and the whole step, at n^3 for n = 32 and 64.
//
// Reference numbers only, like the rest of spade_bench (bench_core.cpp's
// header gives the policy): nothing reads or gates on them. Each family runs
// on an open box holding the Taylor-Green field, so the boundary branches
// run as they do in the drone box. Run in the foreground:
//   build-ninja/release/bin/spade_bench.exe --benchmark_filter=Airflow

#include <benchmark/benchmark.h>

#include <cmath>
#include <cstdint>
#include <numbers>

#include "core/flow_volume.hpp"
#include "physics/airflow/advect.hpp"
#include "physics/airflow/fluid_step.hpp"
#include "physics/airflow/projection.hpp"
#include "physics/airflow/publish.hpp"
#include "physics/airflow/viscous.hpp"

namespace {

using namespace spade::physics::airflow;

struct Case {
    DomainBc bc;  // every face open: the drone box's still air
    MacGrid g;
    FluidConfig cfg;
    FluidScratch s;
};

[[nodiscard]] Case make_case(uint32_t n) {
    Case c;
    const float dx = static_cast<float>(2.0 * std::numbers::pi / n);
    c.g = *make_grid({n, n, n, dx}, c.bc);
    for_each_sample(c.g.shape, 0, [&](std::size_t idx, glm::vec3 x) {
        c.g.u[idx] = static_cast<float>(std::sin(double(x.x)) * std::cos(double(x.y)) * std::cos(double(x.z)));
    });
    for_each_sample(c.g.shape, 1, [&](std::size_t idx, glm::vec3 x) {
        c.g.v[idx] = static_cast<float>(-std::cos(double(x.x)) * std::sin(double(x.y)) * std::cos(double(x.z)));
    });
    c.cfg = FluidConfig{.rho = 1.0f, .nu = 0.01f, .h = 0.5f * dx, .n_v = 2, .visc_sweeps = 8};
    c.s = *make_fluid_scratch(c.g, c.bc, c.cfg);
    return c;
}

void BM_AirflowAdvect(benchmark::State& state) {
    Case c = make_case(static_cast<uint32_t>(state.range(0)));
    for (auto _ : state) {
        advect_velocity(c.g, c.s.next, c.bc, c.cfg.h, c.s.advect);
        benchmark::DoNotOptimize(c.s.next.u.data());
    }
}
BENCHMARK(BM_AirflowAdvect)->Arg(32)->Arg(64)->Unit(benchmark::kMillisecond);

void BM_AirflowDiffuse(benchmark::State& state) {
    Case c = make_case(static_cast<uint32_t>(state.range(0)));
    for (auto _ : state) {
        diffuse(c.g, c.bc, c.cfg.nu, c.cfg.h, c.cfg.visc_sweeps, c.s.rhs);
        benchmark::DoNotOptimize(c.g.u.data());
    }
}
BENCHMARK(BM_AirflowDiffuse)->Arg(32)->Arg(64)->Unit(benchmark::kMillisecond);

void BM_AirflowProject(benchmark::State& state) {
    Case c = make_case(static_cast<uint32_t>(state.range(0)));
    for (auto _ : state) {
        project(c.g, c.bc, c.cfg.rho, c.cfg.h, c.cfg.n_v, c.s.projection);
        benchmark::DoNotOptimize(c.g.p.data());
    }
}
BENCHMARK(BM_AirflowProject)->Arg(32)->Arg(64)->Unit(benchmark::kMillisecond);

void BM_AirflowPublish(benchmark::State& state) {
    Case c = make_case(static_cast<uint32_t>(state.range(0)));
    spade::FlowVolume vol;
    for (auto _ : state) {
        benchmark::DoNotOptimize(publish_render_volume(c.g, {}, vol));
    }
}
BENCHMARK(BM_AirflowPublish)->Arg(32)->Arg(64)->Unit(benchmark::kMillisecond);

void BM_AirflowStep(benchmark::State& state) {
    Case c = make_case(static_cast<uint32_t>(state.range(0)));
    for (auto _ : state) {
        benchmark::DoNotOptimize(fluid_step(c.g, c.bc, c.cfg, c.s));
    }
}
BENCHMARK(BM_AirflowStep)->Arg(32)->Arg(64)->Unit(benchmark::kMillisecond);

// The trilinear sampler alone: velocity_at (three component samples) at points
// a quarter cell off each face, in the advection's own sweep order.
void BM_AirflowVelocityAt(benchmark::State& state) {
    Case c = make_case(static_cast<uint32_t>(state.range(0)));
    const float d = 0.25f * c.g.shape.dx;
    for (auto _ : state) {
        glm::vec3 acc(0.0f);
        for_each_sample(c.g.shape, 0, [&](std::size_t, glm::vec3 x) { acc += velocity_at(c.g, x + d, c.bc); });
        benchmark::DoNotOptimize(acc);
    }
    state.SetItemsProcessed(state.iterations() * int64_t{c.g.shape.nx + 1} * c.g.shape.ny * c.g.shape.nz);
}
BENCHMARK(BM_AirflowVelocityAt)->Arg(32)->Arg(64)->Unit(benchmark::kMillisecond);

}  // namespace
